//
//  neuralnet_classifier.cpp
//  People2D
//
//  Created by Sergey Muratov on 23.03.15.
//  Copyright (c) 2015 Sergey Muratov. All rights reserved.
//

#include "neuralnet_classifier.h"
#include <math.h>
#include <sstream>

neuralnet::neuralnet(LSL_lfeatures_class * features, const char * filename) {
  lfeatures = features;
  configureNetworkFromFile(filename);
}

neuralnet::~neuralnet() {
  delete xkeep;
  delete xgain;
  delete xoffset;
  delete b1;
  delete b2;
  delete IW1;
  delete IW2;
}

void neuralnet::configureNetworkFromFile(const char * fileName) {
  std::ifstream * file = new std::ifstream(fileName);
  
  double hiddenTemp;
  numberFromStream(file, &hiddenTemp);
  hidden = (int)hiddenTemp;
  
  double neuronsKeptTemp;
  numberFromStream(file, &neuronsKeptTemp);
  int neuronsKept = (int)neuronsKeptTemp;
  
  numberFromStream(file, &x_ymin);
  
  numberFromStream(file, &y_ymin);
  numberFromStream(file, &ygain);
  numberFromStream(file, &yoffset);
  
  xkeep = gsl_vector_alloc(neuronsKept);
  vectorFromStream(file, xkeep);
  
  xgain = gsl_vector_alloc(neuronsKept);
  vectorFromStream(file, xgain);
  
  xoffset = gsl_vector_alloc(neuronsKept);
  vectorFromStream(file, xoffset);
  
  b1 = gsl_vector_alloc(hidden);
  vectorFromStream(file, b1);
  
  b2 = gsl_vector_alloc(1);
  vectorFromStream(file, b2);
  
  IW1 = gsl_matrix_alloc(hidden, neuronsKept);
  matrixFromStream(file, IW1);
  
  IW2 = gsl_matrix_alloc(1, hidden);
  matrixFromStream(file, IW2);
}

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
  tansig_apply(b1copy);
  gsl_blas_dgemv(CblasNoTrans, 1.0, IW2, b1copy, 1.0, output);
  logsig_apply(output);
  applyreverseminmax(output);
  
  if (output->data[0] >= 0.99) {
    return true;
  } else {
    return false;
  }
}

#pragma mark - Math helpers

gsl_vector * neuralnet::removeConstantRows(gsl_vector * x) {
  gsl_vector * y = gsl_vector_alloc(xkeep->size);
  for (uint i = 0; i<xkeep->size; ++i) {
    gsl_vector_set(y, i, x->data[(int)xkeep->data[i]-1]);
  }
  return y;
}

void neuralnet::applyminmax(gsl_vector * x) {
  gsl_vector_sub(x, xoffset);
  gsl_vector_mul(x, xgain);
  gsl_vector_add_constant(x, x_ymin);
}

void neuralnet::applyreverseminmax(gsl_vector * y) {
  gsl_vector_add_constant(y, -y_ymin);
  gsl_vector_scale(y, (1.0/ygain));
  gsl_vector_add_constant(y, yoffset);
}

void neuralnet::logsig_apply(gsl_vector * x) {
  for (uint i = 0; i<x->size; ++i) {
    gsl_vector_set(x, i, 1.0/(1.0+exp(-x->data[i])));
  }
}

void neuralnet::tansig_apply(gsl_vector * x) {
  for (uint i = 0; i<x->size; ++i) {
    gsl_vector_set(x, i, (2.0/(1.0+exp(-2.0*x->data[i])))-1.0);
  }
}

#pragma mark - Stream readers

void neuralnet::numberFromStream(std::ifstream * file, double * num) {
  std::string line;
  std::getline(*file, line);
  std::stringstream iss(line);
  std::string number;
  std::getline(iss, number);
  *num = atof(number.c_str());
}

void neuralnet::vectorFromStream(std::ifstream * file, gsl_vector * vec) {
  std::string line;
  uint col = 0;
  while (std::getline(*file, line)) {
    std::stringstream iss(line);
    std::string number;
    while(std::getline(iss, number, ';')) {
      gsl_vector_set(vec, col, atof(number.c_str()));
      col++;
    }
    if (col+1 >= vec->size) {
      return;
    }
  }
}

void neuralnet::matrixFromStream(std::ifstream * file, gsl_matrix * mat) {
  std::string line;
  uint row = 0;
  uint col = 0;
  while (std::getline(*file, line)) {
    std::stringstream iss(line);
    std::string number;
    while(std::getline(iss, number, ';')) {
      gsl_matrix_set(mat, row, col, atof(number.c_str()));
      col++;
    }
    if ((row+1)*(col+1) >= mat->size1*mat->size2) {
      return;
    }
    row++;
    col = 0;
  }
}
