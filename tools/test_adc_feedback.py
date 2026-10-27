"""用主机 GCC 验证真实 C 模块；所有构建产物仅存放于系统临时目录。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]

# 用独立的预期值验证舍入、通道映射和滤波响应，不复制生产代码作为判定函数。
HARNESS = r"""
#include "filter.h"
#include "adc_convert.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

float Test_InlineOtherTu(uint16_t raw_count);

static void ExpectNear(float actual, double expected)
{
    assert(isfinite(actual));
    assert(fabs((double)actual - expected) <= 1e-4 + 1e-5 * fabs(expected));
}

static void AverageRounding(void)
{
    const uint16_t single[] = {17U};
    const uint16_t half[] = {100U, 101U};
    const uint16_t below[] = {1U, 1U, 2U};
    const uint16_t above[] = {1U, 2U, 2U};
    const uint16_t ten[] = {100U, 101U, 102U, 103U, 104U,
                            105U, 106U, 107U, 108U, 109U};
    assert(Filter_AverageU16(single, 1U, UINT16_MAX) == 17U);
    assert(Filter_AverageU16(half, 2U, 1U) == 101U);
    assert(Filter_AverageU16(below, 3U, 1U) == 1U);
    assert(Filter_AverageU16(above, 3U, 1U) == 2U);
    assert(Filter_AverageU16(ten, 10U, 1U) == 105U);
}

static void AverageStrided(void)
{
    const uint16_t samples[] = {10U, 100U, 1000U, 4000U,
                                20U, 200U, 2000U, 3000U,
                                30U, 300U, 3000U, 2000U};
    const uint16_t expected[] = {20U, 200U, 2000U, 3000U};
    uint16_t original[12];
    memcpy(original, samples, sizeof(samples));
    for(uint32_t i = 0U; i < 4U; i++)
    {
        assert(Filter_AverageU16(&samples[i], 3U, 4U) == expected[i]);
    }
    assert(memcmp(original, samples, sizeof(samples)) == 0);
}

static void AverageLimits(void)
{
    uint16_t *samples = malloc((size_t)UINT16_MAX * sizeof(*samples));
    assert(samples != NULL);
    for(uint32_t i = 0U; i < UINT16_MAX; i++)
    {
        samples[i] = UINT16_MAX;
    }
    assert(Filter_AverageU16(samples, UINT16_MAX, 1U) == UINT16_MAX);
    memset(samples, 0, (size_t)UINT16_MAX * sizeof(*samples));
    assert(Filter_AverageU16(samples, UINT16_MAX, 1U) == 0U);
    free(samples);
}

static void AverageWideStride(void)
{
    size_t length = (size_t)UINT16_MAX * 2U + 1U;
    uint16_t *samples = calloc(length, sizeof(*samples));
    assert(samples != NULL);
    samples[0] = 1U;
    samples[UINT16_MAX] = 2U;
    samples[(size_t)UINT16_MAX * 2U] = 6U;
    assert(Filter_AverageU16(samples, 3U, UINT16_MAX) == 3U);
    free(samples);
}

static void ConversionChannels(void)
{
    uint16_t adc_raw[ADC_CHANNEL_COUNT] = {0};
    uint16_t original[ADC_CHANNEL_COUNT];
    float adc_convert[ADC_CHANNEL_COUNT];
    assert(V_CHASSIS == 0 && V_BAT == 1 && I_BAT == 2);
    assert(I_CHASSIS == 3 && V_CAP == 4 && ADC_CHANNEL_COUNT == 5);
    adc_raw[V_CHASSIS] = 1365U;
    adc_raw[V_BAT] = 2730U;
    adc_raw[I_BAT] = 4095U;
    adc_raw[I_CHASSIS] = 0U;
    adc_raw[V_CAP] = 2048U;
    memcpy(original, adc_raw, sizeof(adc_raw));
    assert(AdcConvert_Convert(adc_raw, adc_convert) == adc_convert);
    ExpectNear(adc_convert[V_CHASSIS], 15.4);
    ExpectNear(adc_convert[V_BAT], 24.2);
    ExpectNear(adc_convert[I_BAT], 11.0);
    ExpectNear(adc_convert[I_CHASSIS], -11.0);
    ExpectNear(adc_convert[V_CAP], 18.15443223443223);
    assert(memcmp(original, adc_raw, sizeof(adc_raw)) == 0);
}

static void ConversionEndpoints(void)
{
    uint16_t adc_raw[ADC_CHANNEL_COUNT] = {0};
    float adc_convert[ADC_CHANNEL_COUNT];
    AdcConvert_Convert(adc_raw, adc_convert);
    ExpectNear(adc_convert[V_CHASSIS], 0.0);
    ExpectNear(adc_convert[V_BAT], 0.0);
    ExpectNear(adc_convert[V_CAP], 0.0);
    for(uint32_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
    {
        adc_raw[i] = 4095U;
    }
    AdcConvert_Convert(adc_raw, adc_convert);
    ExpectNear(adc_convert[V_CHASSIS], 46.2);
    ExpectNear(adc_convert[V_BAT], 36.3);
    ExpectNear(adc_convert[V_CAP], 36.3);
    adc_raw[I_BAT] = 2047U;
    adc_raw[I_CHASSIS] = 2048U;
    AdcConvert_Convert(adc_raw, adc_convert);
    ExpectNear(adc_convert[I_BAT], -0.0026862026862);
    ExpectNear(adc_convert[I_CHASSIS], 0.0026862026862);
    assert(adc_convert[I_BAT] < 0.0f && adc_convert[I_CHASSIS] > 0.0f);
}

static void ConversionScalar(void)
{
    ExpectNear(AdcConvert_ConvertValue(1000U, 0.25f, 20.0f), 245.0);
    ExpectNear(AdcConvert_ConvertValue(1000U, 0.25f, 1020.0f), -5.0);
    ExpectNear(AdcConvert_ConvertValue(2048U, 0.5f, 2047.5f), 0.25);
    ExpectNear(AdcConvert_ConvertValue(1000U, 0.25f, 1000.0f), 0.0);
}

static void LowPassEndpoints(void)
{
    ExpectNear(Filter_LowPass(24.0f, 12.0f, 0.0f), 12.0);
    ExpectNear(Filter_LowPass(24.0f, 12.0f, 1.0f), 24.0);
    ExpectNear(Filter_LowPass(24.0f, 24.0f, 0.25f), 24.0);
}

static void LowPassSteps(void)
{
    float history[2] = {0.0f, 8.0f};
    history[0] = Filter_LowPass(8.0f, history[0], 0.5f);
    history[1] = Filter_LowPass(0.0f, history[1], 0.25f);
    ExpectNear(history[0], 4.0);
    ExpectNear(history[1], 6.0);
    history[0] = Filter_LowPass(8.0f, history[0], 0.5f);
    history[1] = Filter_LowPass(0.0f, history[1], 0.25f);
    ExpectNear(history[0], 6.0);
    ExpectNear(history[1], 4.5);
    ExpectNear(Filter_LowPass(-4.0f, 4.0f, 0.75f), -2.0);
    ExpectNear(Filter_LowPass(1.0f, 0.0f, 0.125f), 0.125);
}

static void FeedbackPipeline(void)
{
    uint16_t adc1_samples[40];
    uint16_t adc2_samples[10];
    uint16_t adc_raw[ADC_CHANNEL_COUNT];
    float adc_convert[ADC_CHANNEL_COUNT];
    float history[ADC_CHANNEL_COUNT];
    const uint16_t expected_raw[] = {1005U, 2005U, 2045U, 2055U, 3005U};
    const double expected[] = {1005.0 * 46.2 / 4095.0, 2005.0 * 36.3 / 4095.0,
                               -2.5 * 22.0 / 4095.0, 7.5 * 22.0 / 4095.0,
                               3005.0 * 36.3 / 4095.0};
    const double per_count[] = {46.2 / 4095.0, 36.3 / 4095.0, 22.0 / 4095.0,
                                22.0 / 4095.0, 36.3 / 4095.0};
    for(uint32_t i = 0U; i < 10U; i++)
    {
        adc1_samples[i * 4U] = (uint16_t)(1000U + i);
        adc1_samples[i * 4U + 1U] = (uint16_t)(2000U + i);
        adc1_samples[i * 4U + 2U] = (uint16_t)(2040U + i);
        adc1_samples[i * 4U + 3U] = (uint16_t)(2050U + i);
        adc2_samples[i] = (uint16_t)(3000U + i);
    }
    for(uint32_t i = 0U; i < 4U; i++)
    {
        adc_raw[i] = Filter_AverageU16(&adc1_samples[i], 10U, 4U);
    }
    adc_raw[V_CAP] = Filter_AverageU16(adc2_samples, 10U, 1U);
    AdcConvert_Convert(adc_raw, adc_convert);
    for(uint32_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
    {
        assert(adc_raw[i] == expected_raw[i]);
        ExpectNear(adc_convert[i], expected[i]);
        history[i] = adc_convert[i];
        adc_raw[i] += 10U;
    }
    AdcConvert_Convert(adc_raw, adc_convert);
    for(uint32_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
    {
        history[i] = Filter_LowPass(adc_convert[i], history[i], 0.25f);
        ExpectNear(history[i], expected[i] + 2.5 * per_count[i]);
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if(strcmp(argv[1], "average_rounding") == 0) AverageRounding();
    else if(strcmp(argv[1], "average_strided") == 0) AverageStrided();
    else if(strcmp(argv[1], "average_limits") == 0) AverageLimits();
    else if(strcmp(argv[1], "average_wide_stride") == 0) AverageWideStride();
    else if(strcmp(argv[1], "conversion_channels") == 0) ConversionChannels();
    else if(strcmp(argv[1], "conversion_endpoints") == 0) ConversionEndpoints();
    else if(strcmp(argv[1], "conversion_scalar") == 0) ConversionScalar();
    else if(strcmp(argv[1], "lowpass_endpoints") == 0) LowPassEndpoints();
    else if(strcmp(argv[1], "lowpass_steps") == 0) LowPassSteps();
    else if(strcmp(argv[1], "feedback_pipeline") == 0) FeedbackPipeline();
    else if(strcmp(argv[1], "two_translation_units") == 0)
    {
        ExpectNear(Test_InlineOtherTu(1000U), 125.0);
        ExpectNear(Filter_LowPass(20.0f, 10.0f, 0.5f), 15.0);
        ExpectNear(AdcConvert_ConvertValue(1000U, 0.25f, 20.0f), 245.0);
    }
    else assert(0);
    return 0;
}
"""

OTHER_TRANSLATION_UNIT = r"""
#include "filter.h"
#include "adc_convert.h"

float Test_InlineOtherTu(uint16_t raw_count)
{
    float value = AdcConvert_ConvertValue(raw_count, 0.25f, 20.0f);
    return Filter_LowPass(value, 5.0f, 0.5f);
}
"""


class AdcFeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compiler = shutil.which("gcc")
        if cls.compiler is None:
            raise unittest.SkipTest("NOT_RUN: 未找到 GCC，未执行 C 模块编译和数值测试")
        temporary = tempfile.TemporaryDirectory(prefix="superpower_adc_feedback_")
        cls.addClassCleanup(temporary.cleanup)
        cls.build_dir = Path(temporary.name)
        cls.harness = cls.build_dir / "feedback_test.c"
        cls.other = cls.build_dir / "inline_other.c"
        cls.harness.write_text(HARNESS, encoding="utf-8", newline="\n")
        cls.other.write_text(OTHER_TRANSLATION_UNIT, encoding="utf-8", newline="\n")
        cls.executables = {}

    def run_case(self, name):
        # O0 同样链接两个编译单元，避免仅因优化删去函数而掩盖内联链接问题。
        for optimization in ("-O0", "-O2"):
            with self.subTest(optimization=optimization):
                if optimization not in self.executables:
                    suffix = ".exe" if os.name == "nt" else ""
                    executable = self.build_dir / ("feedback" + optimization + suffix)
                    command = [
                        self.compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic",
                        optimization, "-I", str(ROOT / "User/Algorithm"),
                        "-I", str(ROOT / "User/Bsp"), str(self.harness), str(self.other),
                        str(ROOT / "User/Algorithm/filter.c"),
                        str(ROOT / "User/Bsp/adc_convert.c"), "-lm", "-o", str(executable),
                    ]
                    compiled = subprocess.run(
                        command, cwd=self.build_dir, capture_output=True,
                        encoding="utf-8", errors="replace", timeout=60,
                    )
                    self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
                    self.executables[optimization] = executable
                completed = subprocess.run(
                    [str(self.executables[optimization]), name], cwd=self.build_dir,
                    capture_output=True, encoding="utf-8", errors="replace", timeout=10,
                )
                self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_average_rounding(self):
        self.run_case("average_rounding")

    def test_average_strided(self):
        self.run_case("average_strided")

    def test_average_limits(self):
        self.run_case("average_limits")

    def test_average_wide_stride(self):
        self.run_case("average_wide_stride")

    def test_conversion_channels(self):
        self.run_case("conversion_channels")

    def test_conversion_endpoints(self):
        self.run_case("conversion_endpoints")

    def test_conversion_scalar(self):
        self.run_case("conversion_scalar")

    def test_lowpass_endpoints(self):
        self.run_case("lowpass_endpoints")

    def test_lowpass_steps(self):
        self.run_case("lowpass_steps")

    def test_feedback_pipeline(self):
        self.run_case("feedback_pipeline")

    def test_two_translation_units(self):
        self.run_case("two_translation_units")


if __name__ == "__main__":
    unittest.main()
