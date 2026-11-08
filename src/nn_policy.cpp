#include "nn_policy.h"
#include <math.h>

// int8 weights, float activations. Only ~600 weights and it runs at 25 Hz,
// so the FPU handles this without breaking a sweat.
static void layer(const int8_t *w, float wScale, const float *b,
                  const float *in, int nIn, int nOut, float *out) {
  for (int o = 0; o < nOut; o++) {
    const int8_t *row = w + o * nIn;
    float acc = 0.0f;
    for (int i = 0; i < nIn; i++) acc += (float)row[i] * in[i];
    out[o] = tanhf(acc * wScale + b[o]);
  }
}

void nnPolicy(const float *in, float *out) {
  float h1[NN_H1];
  float h2[NN_H2];
  layer(nn_w1, nn_w1_scale, nn_b1, in, NN_IN, NN_H1, h1);
  layer(nn_w2, nn_w2_scale, nn_b2, h1, NN_H1, NN_H2, h2);
  layer(nn_w3, nn_w3_scale, nn_b3, h2, NN_H2, NN_OUT, out);
}
