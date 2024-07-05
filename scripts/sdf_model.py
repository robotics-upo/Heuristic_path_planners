# MIT License

# Copyright (c) 2020 Vincent Sitzmann
# Modified by: Vasileios Vasilopoulos (vasileios.v@samsung.com)

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import torch
from torch import nn
import numpy as np
import math
import io


########################
# Networks
########################
class Sine(nn.Module):
    """
    Sine non-linearity.
    """

    def __init__(self):
        super().__init__()

    def forward(self, input):
        return torch.sin(10 * input)


class FCNet(nn.Module):
    """
    A fully connected neural network with specific activations.
    """

    def __init__(
            self,
            in_features,
            out_features,
            num_hidden_layers,
            hidden_features,
            outermost_linear = False,
            nonlinearity = 'relu',
            weight_init = None
        ):
        super().__init__()

        self.first_layer_init = None

        # Dictionary that maps nonlinearity name to the respective function, initialization, and, if applicable,
        # special first-layer initialization scheme
        nls_and_inits = {'sine':(Sine(), sine_init, None),
                         'relu':(nn.ReLU(inplace=True), init_weights_normal, None),
                         'sigmoid':(nn.Sigmoid(), init_weights_xavier, None),
                         'tanh':(nn.Tanh(), init_weights_xavier, None),
                         'selu':(nn.SELU(inplace=True), init_weights_selu, None),
                         'softplus':(nn.Softplus(), init_weights_normal, None),
                         'elu':(nn.ELU(inplace=True), init_weights_elu, None)}

        nl, nl_weight_init, first_layer_init = nls_and_inits[nonlinearity]

        if weight_init is not None:  # Overwrite weight init if passed
            self.weight_init = weight_init
        else:
            self.weight_init = nl_weight_init

        self.net = []
        self.net.append(nn.Sequential(nn.Linear(in_features, hidden_features), nl))

        for i in range(num_hidden_layers):
            self.net.append(nn.Sequential(nn.Linear(hidden_features, hidden_features), nl))

        if outermost_linear:
            self.net.append(nn.Sequential(nn.Linear(hidden_features, out_features)))
        else:
            self.net.append(nn.Sequential(nn.Linear(hidden_features, out_features), nl))

        self.net = nn.Sequential(*self.net)
        if self.weight_init is not None:
            self.net.apply(self.weight_init)

        if first_layer_init is not None: # Apply special initialization to first layer, if applicable.
            self.net[0].apply(first_layer_init)

    def forward(self, input):
        return self.net(input)


class HIOSDFNet(nn.Module):
    """
    Global SDF neural network.
    """

    def __init__(
            self,
            in_features = 3,
            out_features = 1,
            type = 'sine',
            hidden_features = 256,
            num_hidden_layers = 4,
        ):
        super().__init__()


        self.net = FCNet(
            in_features = in_features,
            out_features = out_features,
            num_hidden_layers = num_hidden_layers,
            hidden_features = hidden_features,
            outermost_linear = True,
            nonlinearity = type,
        )
        print(self)

    def forward(self, model_input):
        return self.net(model_input)
    

########################
# Initialization methods
########################
def init_weights_normal(m):
    if type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            nn.init.kaiming_normal_(m.weight, a=0.0, nonlinearity='relu', mode='fan_in')


def init_weights_selu(m):
    if type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            nn.init.normal_(m.weight, std=1 / math.sqrt(num_input))


def init_weights_elu(m):
    if type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            nn.init.normal_(m.weight, std=math.sqrt(1.5505188080679277) / math.sqrt(num_input))


def init_weights_xavier(m):
    if type(m) == nn.Linear:
        if hasattr(m, 'weight'):
            nn.init.xavier_normal_(m.weight)


def sine_init(m):
    with torch.no_grad():
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            m.weight.uniform_(-np.sqrt(6 / num_input) / 10, np.sqrt(6 / num_input) / 10)


def first_layer_sine_init(m):
    with torch.no_grad():
        if hasattr(m, 'weight'):
            num_input = m.weight.size(-1)
            m.weight.uniform_(-1 / num_input, 1 / num_input)


################################################
# Model instantiation and prediction function
################################################
sdf_model = HIOSDFNet()
sdf_model.eval()

def load_weights(weights):
    buffer = io.BytesIO(weights)
    state_dict = torch.load(buffer)
    sdf_model.load_state_dict(state_dict)
    sdf_model.eval()

def predict(x, y, z):
    with torch.no_grad():
        input_tensor = torch.tensor([x, y, z], dtype=torch.float32)
        return sdf_model(input_tensor).item()
