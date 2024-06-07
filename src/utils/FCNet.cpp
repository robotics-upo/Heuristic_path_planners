#include "utils/FCNet.hpp"
#include <cmath>

// Sine non-linearity
torch::Tensor Sine::forward(torch::Tensor x) {
    return torch::sin(10 * x);
}

// Initialization methods
void HIOSDFNet::sine_init(torch::nn::Module& module) {
    if (auto* linear = module.as<torch::nn::Linear>()) {
        auto num_input = linear->weight.size(1);
        auto weight = torch::empty_like(linear->weight);
        weight.uniform_(-std::sqrt(6.0 / num_input) / 10, std::sqrt(6.0 / num_input) / 10);
        linear->weight = weight;
    }
}

void HIOSDFNet::first_layer_sine_init(torch::nn::Module& m) {
    if (auto* linear = m.as<torch::nn::Linear>()) {
        int64_t num_input = linear->weight.size(1);
        linear->weight.uniform_(-1.0 / num_input, 1.0 / num_input);
    }
}

// HIOSDFNet class
HIOSDFNet::HIOSDFNet(int64_t in_features, int64_t out_features,
                     int64_t hidden_features, int64_t num_hidden_layers) {

    auto nl = register_module("sine", std::make_shared<Sine>());

    net->push_back(torch::nn::Linear(in_features,hidden_features));
    net->push_back(*nl);

    for (int64_t i = 0; i < num_hidden_layers; ++i) {
    net->push_back(torch::nn::Linear(hidden_features,hidden_features));
    net->push_back(*nl);
    }

    net->push_back(torch::nn::Linear(hidden_features, out_features));

    // Apply weight initialization
    net->apply([this](torch::nn::Module& m) { sine_init(m); });
    net[0]->apply([this](torch::nn::Module& m) { first_layer_sine_init(m); });

    register_module("net", net);
}

torch::Tensor HIOSDFNet::forward(torch::Tensor x) {
    return net->forward(x);
}

void HIOSDFNet::eval() {
    torch::nn::Module::eval();
}

// void HIOSDFNet::load_weights_from_tensor(torch::Tensor& tensor) {
//     auto data_ptr = tensor.data_ptr<uint8_t>();
//     std::istringstream in_stream(std::string(data_ptr, data_ptr + tensor.numel()));
//     torch::serialize::InputArchive archive;
//     archive.load_from(in_stream);
//     this->load(archive);
// }
void HIOSDFNet::load_weights_from_tensor(torch::Tensor& tensor) {
    // Detach tensor from the computation graph and ensure it does not require gradients
    tensor = tensor.detach().cpu().contiguous();
    tensor.set_requires_grad(false);

    auto params = this->named_parameters();
    size_t offset = 0;

    for (auto& param : params) {
        auto& p = param.value();
        auto numel = p.numel();
        auto slice = tensor.narrow(0, offset, numel);
        slice = slice.view(p.sizes()).clone(); // Clone to ensure slice does not share storage

        // Ensure the slice does not require gradients
        slice.set_requires_grad(false);

        // Ensure the parameter tensor does not require gradients temporarily
        bool requires_grad_backup = p.requires_grad();
        p.set_requires_grad(false);

        p.copy_(slice);

        // Restore the original requires_grad state
        p.set_requires_grad(requires_grad_backup);

        offset += numel;
    }
}