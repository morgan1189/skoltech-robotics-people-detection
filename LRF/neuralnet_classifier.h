//
//  neuralnet_classifier.h
//  People2D
//
//  Created by Sergey Muratov on 23.03.15.
//  Copyright (c) 2015 Sergey Muratov. All rights reserved.
//

#ifndef __People2D__neuralnet_classifier__
#define __People2D__neuralnet_classifier__

#include <stdio.h>
#include <vector>
#include <string>
#include "lfeatures.cpp"
#include <gsl_blas.h>

typedef std::vector<double> d_vec;

class neuralnet {
  private:
  gsl_vector * b1;
  gsl_vector * b2;
  gsl_matrix * IW1;
  gsl_matrix * IW2;
  gsl_vector * xoffset;
  gsl_vector * gain;
  gsl_vector * xkeep;
  double ymin;
  gsl_vector * removeConstantRows(gsl_vector * x);
  void applyminmax(gsl_vector * x);
  void tansig_apply(gsl_vector * x);
  void logsig_apply(gsl_vector * x);
  bool isALeg(gsl_vector * features);
  public:
  LSL_lfeatures_class * lfeatures;
  void configureNetworkFromFile(std::string fileName);
  std::vector<LSL_Point3D_container> classifyClusters(std::vector<LSL_Point3D_container> clusters);
};


#endif /* defined(__People2D__neuralnet_classifier__) */
