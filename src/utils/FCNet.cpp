#include "utils/FCNet.hpp"
#include <cmath>
#include <fstream>

// Sine non-linearity
torch::Tensor Sine::forward(torch::Tensor x) {
    return torch::sin(10 * x);
}

// Custom Linear + Sine module
struct LinearSine : torch::nn::Module {
    LinearSine(int64_t in_features, int64_t out_features) {
        linear = register_module("linear", torch::nn::Linear(in_features, out_features));
        sine = register_module("sine", std::make_shared<Sine>());
    }

    torch::Tensor forward(torch::Tensor x) {
        x = linear->forward(x);
        x = sine->forward(x);
        return x;
    }

    torch::nn::Linear linear{nullptr};
    std::shared_ptr<Sine> sine{nullptr};
};

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

    // First layer
    net->push_back(register_module("first_layer", std::make_shared<LinearSine>(in_features, hidden_features)));

    // Hidden layers
    for (int64_t i = 0; i < num_hidden_layers; ++i) {
        net->push_back(register_module("hidden_layer_" + std::to_string(i), std::make_shared<LinearSine>(hidden_features, hidden_features)));
    }

    // Output layer
    net->push_back(register_module("output_layer", torch::nn::Linear(hidden_features, out_features)));

    //for (int64_t i = 0; i < num_hidden_layers; ++i) {
    //net->push_back(torch::nn::Linear(hidden_features,hidden_features));
    //net->push_back(*nl);
    //}

    //net->push_back(torch::nn::Linear(hidden_features, out_features));

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

    auto params = net->named_parameters(); // Only target the "net" module
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


void HIOSDFNet::saveWeightsToFileOriginal(const std::string &filename, const std::vector<uint8_t> &weights) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto &weight : weights) {
            file << static_cast<float>(weight) << "\n";
        }
        file.close();
        ROS_INFO("Model weights saved (original).");
    } else {
        ROS_ERROR("Unable to open file: %s", filename.c_str());
    }
}

void HIOSDFNet::saveWeightsToFileNew(const std::string &filename, const torch::Tensor &tensor) {
    std::ofstream file(filename);
    if (file.is_open()) {
        auto tensor_data = tensor.accessor<float, 1>();
        for (int64_t i = 0; i < tensor.size(0); ++i) {
            file << tensor_data[i] << "\n";
        }
        file.close();
        ROS_INFO("Model weights saved (new).");
    } else {
        ROS_ERROR("Unable to open file: %s", filename.c_str());
    }
}

torch::Tensor HIOSDFNet::getWeightsAsTensor() const {
    // Vector to store all parameter tensors
    std::vector<torch::Tensor> param_tensors;

    // Iterate over all named parameters in the network
    for (const auto& param : net->named_parameters()) {
        param_tensors.push_back(param.value().view(-1)); // Flatten each parameter tensor
    }

    // Concatenate all parameter tensors into a single tensor
    return torch::cat(param_tensors);
}

void HIOSDFNet::forwardSave(const std::string &filename, torch::Tensor& input_tensor) {

    std::ofstream file(filename, std::ios::app);
    if (file.is_open()) {
        torch::Tensor output_tensor = net->forward(input_tensor);
        file << "Net value for (x,y,z) =" << input_tensor << " ==> " << output_tensor.item<float>() << "\n";
        
        file.close();
        ROS_INFO("forwardSave done");
    } else {
        ROS_ERROR("Unable to open file: %s", filename.c_str());
    }
}


// void HIOSDFNet::save_net_structure(const std::string& filename, const std::string& prefix) const {
//     std::ofstream file(filename);
//     if (file.is_open()) {
//         for (const auto& param : this->named_parameters(/*recurse=*/false)) {
//             file << prefix << param.key() << ": " << param.value().sizes() << "\n";
//         }
//         file.close();
//         ROS_INFO("Network structure saved to file: %s", filename.c_str());
//     } else {
//         ROS_ERROR("Unable to open file: %s", filename.c_str());
//     }
// }

void HIOSDFNet::save_net_structure(const std::string& filename, const std::string& prefix) const {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& module : named_modules()) {
            if (module.value().get() != this) { // Avoid printing the root module itself
                file << prefix << module.key() << " (" << module.value()->name() << ")\n";
            }
        }

        for (const auto& param : named_parameters(/*recurse=*/true)) {
            file << prefix << param.key() << ": " << param.value().sizes() << "\n";
        }
        file.close();
        ROS_INFO("Network structure saved to file: %s", filename.c_str());
    } else {
        ROS_ERROR("Unable to open file: %s", filename.c_str());
    }
}

void HIOSDFNet::save_params_by_layer(const std::string& filename){
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Unable to open file for writing!" << std::endl;
        return;
    }

    // Write the parameters to the file
    for (const auto& pair : net->named_parameters()) {
        outfile << pair.key() << ":\n" << pair.value() << "\n\n";
    }

    outfile.close();
    }

