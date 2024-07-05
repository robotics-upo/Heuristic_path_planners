#ifndef HIOSDFNET_HPP
#define HIOSDFNET_HPP

#include <torch/torch.h>
#include <ros/ros.h>
#include <fstream>


// Sine non-linearity
struct Sine : torch::nn::Module {
    torch::Tensor forward(torch::Tensor x);
};

// HIOSDFNet class
class HIOSDFNet : public torch::nn::Module {
public:
    HIOSDFNet(int64_t in_features = 3, int64_t out_features = 1,
              int64_t hidden_features = 256, int64_t num_hidden_layers = 4);

    torch::Tensor forward(torch::Tensor x);
    void eval();
    void load_weights_from_tensor(torch::Tensor& tensor);
    void saveWeightsToFileOriginal(const std::string &filename, const std::vector<uint8_t> &weights);
    void saveWeightsToFileNew(const std::string &filename, const torch::Tensor &tensor);
    torch::Tensor getWeightsAsTensor() const;
    void forwardSave(const std::string &filename, torch::Tensor& input_tensor);
    void save_net_structure(const std::string& filename, const std::string& prefix) const;
    void save_params_by_layer(const std::string& filename);
    
private:
    torch::nn::Sequential net;

    void sine_init(torch::nn::Module& module);
    void first_layer_sine_init(torch::nn::Module& m);
};

#endif // HIOSDFNET_HPP

