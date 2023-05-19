#pragma once

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "array.h"

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <vector>

//--------------------------------------

// Very basic feed-forward neural network
// class. Assumes relu activation on every
// layer except the last. Also includes 
// normalization and denormalization to make 
// preparing the inputs and outputs easier.
struct nnet
{   
    array1d<float> input_mean;
    array1d<float> input_std;
    array1d<float> output_mean;
    array1d<float> output_std;
    std::vector<array2d<float>> weights;
    std::vector<array1d<float>> biases;
};

void nnet_load(nnet& nn, const char* filename);

//--------------------------------------

static inline void nnet_layer_normalize(
    slice1d<float> output,
    const slice1d<float> mean,
    const slice1d<float> std);

static inline void nnet_layer_denormalize(
    slice1d<float> output,
    const slice1d<float> mean,
    const slice1d<float> std);

// This appears to be the fastest way I found to do basic
// matmul on the CPU. It takes advantage of activations set
// to zero due to relu and compiles well to SIMD. It's important
// for performance that the pointers are labelled restrict.
static inline void nnet_layer_linear(
    slice1d<float> output,
    const slice1d<float> input,
    const slice2d<float> weights,
    const slice1d<float> biases);

static inline void nnet_layer_relu(slice1d<float> output);

//--------------------------------------

// Basic class that can be used to pre-allocate 
// the storage required to do network inference 
// (i.e. activations).
struct nnet_evaluation
{
    std::vector<array1d<float>> layers;
    
    // Resize for a given network
    void resize(const nnet& nn)
    {
        layers.resize(nn.weights.size() + 1);
        layers.front().resize(nn.weights.front().rows);
      
        for (int i = 0; i < nn.weights.size(); i++)
        {
            layers[i+1].resize(nn.weights[i].cols);            
        }
    }
};

// Neural Network evaluation function. Assumes input 
// has been placed in first layer of `evaulation` 
// object. Puts result in the last layer of the 
// `evaluation` object.
void nnet_evaluate(
    nnet_evaluation& evaluation,
    const nnet& nn);

