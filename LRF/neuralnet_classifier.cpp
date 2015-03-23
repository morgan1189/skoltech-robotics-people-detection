//
//  neuralnet_classifier.cpp
//  People2D
//
//  Created by Sergey Muratov on 23.03.15.
//  Copyright (c) 2015 Sergey Muratov. All rights reserved.
//

#include "neuralnet_classifier.h"
#include <math.h>

void neuralnet::configureNetworkFromFile(std::string fileName = "") {
  if (fileName.length() == 0) {
    // Using default parameters
  }
}

// Just a comment to test Slack integration

std::vector<LSL_Point3D_container> neuralnet::classifyClusters(std::vector<LSL_Point3D_container> clusters) {
  std::vector <LSL_Point3D_container> legs;
  std::vector <std::vector<Real>> descriptor;
  lfeatures->compute_descriptor(clusters, descriptor);
  for (uint w=0; w < descriptor.size(); w++) {
    gsl_vector * features = gsl_vector_alloc(descriptor[w].size());
    for (uint m=0; m < descriptor[w].size(); m++) {
      gsl_vector_set(features, m, descriptor[w][m]);
    }
    if(isALeg(features)) {
      legs.push_back(clusters.at(w));
    }
  }
  return legs;
}

bool neuralnet::isALeg(gsl_vector * features) {
  gsl_vector * b1copy = gsl_vector_alloc(b1->size);
  gsl_vector_memcpy(b1copy, b1);
  
  gsl_vector * output = gsl_vector_alloc(b2->size);
  gsl_vector_memcpy(output, b2);
  
  gsl_vector * xp1 = removeConstantRows(features);
  applyminmax(xp1);
  gsl_blas_dgemv(CblasNoTrans, 1.0, IW1, xp1, 1.0, b1copy);
  gsl_blas_dgemv(CblasNoTrans, 1.0, IW2, b1copy, 1.0, output);
  
  if (output->data[0] >= 0.5) {
    return true;
  } else {
    return false;
  }
}

#pragma mark - Math helpers

gsl_vector * neuralnet::removeConstantRows(gsl_vector * x) {
  gsl_vector * y = gsl_vector_alloc(xkeep->size);
  for (uint i = 0; i<xkeep->size; ++i) {
    gsl_vector_set(y, i, x->data[(int)xkeep->data[i]]);
  }
  return y;
}

void neuralnet::applyminmax(gsl_vector * x) {
  gsl_vector_sub(x, xoffset);
  gsl_vector_mul(x, gain);
  gsl_vector_add_constant(x, ymin);
}

void neuralnet::logsig_apply(gsl_vector * x) {
  for (uint i = 0; i<x->size; ++i) {
    gsl_vector_set(x, i, 1.0/(1.0+exp(x->data[i])));
  }
}

void neuralnet::tansig_apply(gsl_vector * x) {
  for (uint i = 0; i<x->size; ++i) {
    gsl_vector_set(x, i, (2.0/(1.0+exp(-2.0*x->data[i])))-1.0);
  }
}
