#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void require_shape(const std::vector<int64_t>& actual,
                   const std::vector<int64_t>& expected,
                   const std::string& label) {
  require(actual == expected, label + " shape mismatch");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "usage: cpp_onnx_smoke_test MODEL.onnx");
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "go2_legacy_compat_smoke_test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    Ort::Session session(env, argv[1], options);
    Ort::AllocatorWithDefaultOptions allocator;

    require(session.GetInputCount() == 2, "expected exactly two inputs");
    require(session.GetOutputCount() == 1, "expected exactly one output");

    const auto input0 = session.GetInputNameAllocated(0, allocator);
    const auto input1 = session.GetInputNameAllocated(1, allocator);
    const auto output0 = session.GetOutputNameAllocated(0, allocator);
    require(std::strcmp(input0.get(), "obs") == 0, "input 0 must be obs");
    require(std::strcmp(input1.get(), "history_obs") == 0,
            "input 1 must be history_obs");
    require(std::strcmp(output0.get(), "actions") == 0,
            "output must be actions");

    require_shape(session.GetInputTypeInfo(0)
                      .GetTensorTypeAndShapeInfo()
                      .GetShape(),
                  {1, 45}, "obs");
    require_shape(session.GetInputTypeInfo(1)
                      .GetTensorTypeAndShapeInfo()
                      .GetShape(),
                  {1, 10, 45}, "history_obs");
    require_shape(session.GetOutputTypeInfo(0)
                      .GetTensorTypeAndShapeInfo()
                      .GetShape(),
                  {1, 12}, "actions");

    std::vector<float> obs(45, 0.0F);
    obs[5] = -1.0F;
    std::vector<float> history(10 * 45, 0.0F);
    const std::vector<int64_t> obs_shape{1, 45};
    const std::vector<int64_t> history_shape{1, 10, 45};
    const auto memory =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto obs_tensor = Ort::Value::CreateTensor<float>(
        memory, obs.data(), obs.size(), obs_shape.data(), obs_shape.size());
    auto history_tensor = Ort::Value::CreateTensor<float>(
        memory, history.data(), history.size(), history_shape.data(),
        history_shape.size());

    const char* input_names[] = {input0.get(), input1.get()};
    const char* output_names[] = {"actions"};
    std::vector<Ort::Value> inputs;
    inputs.emplace_back(std::move(obs_tensor));
    inputs.emplace_back(std::move(history_tensor));
    auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names,
                               inputs.data(), inputs.size(), output_names, 1);
    require(outputs.size() == 1, "runtime returned the wrong output count");
    const auto output_info = outputs[0].GetTensorTypeAndShapeInfo();
    require(output_info.GetElementCount() == 12,
            "runtime output must contain 12 floats");
    const float* actions = outputs[0].GetTensorData<float>();
    for (std::size_t i = 0; i < 12; ++i) {
      require(std::isfinite(actions[i]), "runtime returned non-finite action");
    }

    std::cout << "PASS C++ ONNX Runtime smoke test\n";
    std::cout << "inputs=" << input0.get() << "," << input1.get()
              << " output=" << output0.get() << " actions=";
    for (std::size_t i = 0; i < 12; ++i) {
      std::cout << (i == 0 ? "" : ",") << actions[i];
    }
    std::cout << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << "\n";
    return 1;
  }
}
