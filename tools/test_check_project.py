"""规范检查工具的行为测试；所有样例均写入系统临时目录。"""

import contextlib
import hashlib
import importlib.util
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch


CHECKER_PATH = Path(__file__).with_name("check_project.py")


class ProjectCheckTests(unittest.TestCase):
    """验证可观测结果，不修改实际工程或 Git 索引。"""

    @classmethod
    def setUpClass(cls):
        if not CHECKER_PATH.is_file():
            return
        spec = importlib.util.spec_from_file_location("check_project", CHECKER_PATH)
        cls.checker = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.checker)

    def setUp(self):
        self.assertTrue(CHECKER_PATH.is_file(), "规范检查工具尚未实现")
        self.temporary = tempfile.TemporaryDirectory(prefix="supercap-check-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def write_sample(self, relative, content):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        return path

    def messages(self, relative, content):
        path = self.write_sample(relative, content)
        return self.checker.check_file(path, self.root)

    def run_main(self, *arguments):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            result = self.checker.main(["--root", str(self.root), *arguments])
        return result, output.getvalue()

    def test_utf8_chinese_and_space_in_path_are_accepted(self):
        messages = self.messages("docs/中文 说明.md", "# 工程说明\n".encode("utf-8"))
        self.assertEqual(messages, [])

    def test_gb2312_is_rejected_with_path_and_line(self):
        messages = self.messages("docs/旧编码.md", "标题\n中文注释\n".encode("gb2312"))
        self.assertTrue(any("ENCODING_UTF8" in item for item in messages), messages)
        self.assertTrue(all("docs/旧编码.md:1:" in item for item in messages), messages)

    def test_utf8_bom_is_rejected(self):
        messages = self.messages("docs/bom.md", b"\xef\xbb\xbftext\n")
        self.assertTrue(any("BOM" in item for item in messages), messages)

    def test_tab_in_c_and_header_is_rejected(self):
        for suffix in ("c", "h"):
            with self.subTest(suffix=suffix):
                messages = self.messages(f"User/App/example.{suffix}", b"void Example_Init(void);\n\t/* comment */\n")
                self.assertTrue(any(":2:1:" in item and "TAB" in item for item in messages), messages)

    def test_markdown_tab_is_not_treated_as_c_indentation(self):
        self.assertEqual(self.messages("docs/example.md", b"\tcode block\n"), [])

    def test_trailing_spaces_are_rejected(self):
        messages = self.messages("docs/example.md", b"title\ntext  \n")
        self.assertTrue(any(":2:" in item and "TRAILING_WHITESPACE" in item for item in messages), messages)

    def test_missing_final_newline_and_empty_file_are_rejected(self):
        for content in (b"text", b""):
            with self.subTest(content=content):
                messages = self.messages("docs/example.md", content)
                self.assertTrue(any("FINAL_NEWLINE" in item for item in messages), messages)

    def test_crlf_is_rejected_for_managed_files(self):
        messages = self.messages("User/App/example.c", b"/* comment */\r\n")
        self.assertTrue(any("LINE_ENDING" in item for item in messages), messages)

    def test_scan_excludes_protected_and_build_files(self):
        excluded = (
            "Core/Src/main.c", "Drivers/vendor.h", "MDK-ARM/startup.s",
            "references/old.c", "2026superpower.ioc", "User/config.ioc",
            "User/build/output.c", "User/Core/generated.c", "tools/__pycache__/cache.py",
            "unrelated.c", "docs/plot.png",
        )
        for relative in excluded:
            self.write_sample(relative, b"not managed")
        managed = self.write_sample("User/App/example.c", b"/* example */\n")
        self.assertEqual(self.checker.collect_paths(self.root), [managed])

    def test_generated_header_inside_user_is_excluded(self):
        self.write_sample("User/App/generated.c", b"/* USER CODE BEGIN Header */\n")
        self.assertEqual(self.checker.collect_paths(self.root), [])

    def test_missing_managed_root_directory_is_allowed(self):
        self.assertEqual(self.checker.collect_paths(self.root), [])

    def test_check_does_not_rewrite_invalid_content(self):
        path = self.write_sample("User/App/example.c", b"\xef\xbb\xbf\tbad  \r\n")
        before = hashlib.sha256(path.read_bytes()).digest()
        self.checker.check_file(path, self.root)
        self.assertEqual(hashlib.sha256(path.read_bytes()).digest(), before)

    def test_default_cli_checks_all_managed_files_and_reports_success(self):
        self.write_sample("README.md", "# 超电工程\n".encode("utf-8"))
        self.write_sample("templates/example.h", b"/* example */\n")
        result, output = self.run_main()
        self.assertEqual(result, 0, output)
        self.assertIn("PASS", output)
        self.assertIn("2", output)

    def test_cli_returns_one_for_content_violations(self):
        self.write_sample("README.md", b"trailing space \n")
        result, output = self.run_main()
        self.assertEqual(result, 1, output)
        self.assertIn("TRAILING_WHITESPACE", output)
        self.assertNotIn("PASS", output)

    def test_no_selected_files_is_reported_explicitly(self):
        result, output = self.run_main()
        self.assertEqual(result, 0, output)
        self.assertIn("NO_FILES", output)
        self.assertNotIn("PASS", output)

    def test_changed_selection_unions_worktree_index_and_untracked_paths(self):
        changed = self.write_sample("docs/中文 说明.md", b"text\n")
        staged = self.write_sample("User/App/staged.c", b"/* text */\n")
        untracked = self.write_sample("tools/new.py", b"# text\n")
        self.write_sample("Core/Src/main.c", b"not managed")
        self.write_sample("docs/unchanged.md", b"text\n")
        # Git 是外部只读数据源；模拟 NUL 分隔输出，不在测试中执行暂存或提交。
        responses = [
            "docs/中文 说明.md\0Core/Src/main.c\0docs/deleted.md\0".encode("utf-8"),
            b"User/App/staged.c\0docs/\xe4\xb8\xad\xe6\x96\x87 \xe8\xaf\xb4\xe6\x98\x8e.md\0",
            b"tools/new.py\0",
        ]
        with patch.object(self.checker, "git_output", side_effect=responses):
            actual = self.checker.collect_paths(self.root, changed=True)
        self.assertEqual(set(actual), {changed, staged, untracked})

    def test_changed_cli_reports_git_failure_as_tool_error(self):
        with patch.object(self.checker, "git_output", side_effect=RuntimeError("Git unavailable")):
            result, output = self.run_main("--changed")
        self.assertEqual(result, 2, output)
        self.assertIn("Git unavailable", output)
        self.assertNotIn("PASS", output)

    def test_missing_formatter_is_not_a_pass(self):
        self.write_sample("templates/example.h", b"/* example */\n")
        with patch.object(self.checker.shutil, "which", return_value=None):
            result, output = self.run_main("--format")
        self.assertEqual(result, 2, output)
        self.assertIn("NOT_RUN", output)
        self.assertNotIn("PASS", output)

    def test_formatter_failure_returns_nonzero_and_preserves_source(self):
        path = self.write_sample("templates/example.h", b"/* example */\n")
        self.write_sample(".clang-format", b"BasedOnStyle: LLVM\n")
        before = path.read_bytes()
        failure = subprocess.CompletedProcess([], 1, stdout="", stderr="format violation")
        with patch.object(self.checker.shutil, "which", return_value="test-clang-format"):
            with patch.object(self.checker.subprocess, "run", return_value=failure):
                result, output = self.run_main("--format")
        self.assertEqual(result, 1, output)
        self.assertIn("format violation", output)
        self.assertEqual(path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
