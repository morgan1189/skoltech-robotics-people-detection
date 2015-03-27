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
#include <gsl/gsl_blas.h>
#include "people2D_engine.hpp"

class neuralnet {
  private:
    int hidden = 10;
    gsl_vector * xkeep;
    gsl_vector * xoffset;
    gsl_vector * xgain;
    double x_ymin;
  
    double yoffset;
    double ygain;
    double y_ymin;
  
    gsl_vector * b1;
    gsl_vector * b2;
    gsl_matrix * IW1;
    gsl_matrix * IW2;
  
    LSL_lfeatures_class * lfeatures;

    bool isALeg(gsl_vector * features);
    void configureNetworkFromFile(const char * fileName);
  
    gsl_vector * removeConstantRows(gsl_vector * x);
    void applyminmax(gsl_vector * x);
    void applyreverseminmax(gsl_vector * y);
    void tansig_apply(gsl_vector * x);
    void logsig_apply(gsl_vector * x);
    void vectorFromStream(std::ifstream * file, gsl_vector * vec);
    void matrixFromStream(std::ifstream * file, gsl_matrix * mat);
    void numberFromStream(std::ifstream * file, double * num);
  public:
    neuralnet(LSL_lfeatures_class * lfeatures, const char * fileName = "/Users/Sergey/LegsNet/netparams.txt");
    ~neuralnet();
    std::vector<LSL_Point3D_container> classifyClusters(std::vector<LSL_Point3D_container> clusters);
};


#endif /* defined(__People2D__neuralnet_classifier__) */
