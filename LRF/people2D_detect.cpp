/*****************************************************************
 *
 * This file is part of the People2D project
 *
 * People2D Copyright (c) 2011 Luciano Spinello
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 3.0)" 
 * and is copyrighted by Luciano Spinello
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/3.0/
 * 
 * People2D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/



#include "people2D_engine.hpp"
#include "neuralnet_classifier.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"
#include "urg_c/urg_detect_os.h"


////~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ 

void prg_info(void)
{
		printf(
	"Detects people in 2D range data\n"
	"options:\n"
	"-i input scan file\n"
	"-m model file\n"	
	"-d segmentation distance in m\n"
	"-pr consider the annotations in the file and compute pr curve\n"
	"-fx (valids: f0,f1,f2) feature set mix (default 0:all)\n"
	"-o output file\n"
	"-s ignores all params and writes segmentation only on disk\n"
	"-S do not run sanity checks on data (DISCOURAGED but faster)\n"
	"-B benchmark mode, no verbosity, no result save\n"
	"-v (1-3) verbosity (default = 0)\n"
	);
}

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ 


int parse_command_line(int argc, char **argv, sw_param_str *sw_param)
{
	int i;
	
	sw_param -> sanity = 1;
	sw_param -> segonly = 0;
	sw_param -> featuremix = 0;
	sw_param -> verbosity = 0;
	sw_param -> precall = 0;
	sw_param -> benchmark = 0;
	
	if(argc < 5)
	{
		prg_info();
		return(0);
	}
		
	for(i=0; i<argc; i++)
	{
		
	 
		if(!strcmp(argv[i], "-i"))		
				sw_param -> inputfile = argv[i+1];
		
		if(!strcmp(argv[i], "-o"))		
				sw_param -> outputfile = argv[i+1];

		if(!strcmp(argv[i], "-m"))		
				sw_param -> modelfile = argv[i+1];
		
		if(!strcmp(argv[i], "-d"))		
				sw_param -> dseg = atof(argv[i+1]);

		if(!strcmp(argv[i], "-f1"))		
				sw_param -> featuremix = 1;

		if(!strcmp(argv[i], "-f2"))		
				sw_param -> featuremix = 2;

		if(!strcmp(argv[i], "-f3"))		
				sw_param -> featuremix = 3;
				
		if(!strcmp(argv[i], "-s"))		
				sw_param -> segonly = 1;

		if(!strcmp(argv[i], "-v1"))		
				sw_param -> verbosity = 1;

		if(!strcmp(argv[i], "-v2"))		
				sw_param -> verbosity = 2;

		if(!strcmp(argv[i], "-v3"))		
				sw_param -> verbosity = 3;

		if(!strcmp(argv[i], "-S"))		
				sw_param -> sanity = 0;

		if(!strcmp(argv[i], "-B"))		
				sw_param -> benchmark = 1;

		if(!strcmp(argv[i], "-pr"))		
				sw_param -> precall = 1;

	}
	
	if(!sw_param -> inputfile.size() || !sw_param -> outputfile.size() || sw_param -> dseg == 0 )
	{
		prg_info();
		exit(0);
	}
			
	printf("[PAR] Input file: %s\n",sw_param -> inputfile.c_str());	
	printf("[PAR] Output file: %s\n",sw_param -> outputfile.c_str());	
	printf("[PAR] Model file: %s\n",sw_param -> modelfile.c_str());	
	if(sw_param -> benchmark)
	{
		printf("** Benchmark mode **\n");	
		sw_param -> verbosity = 0;
		sw_param -> sanity = 0;
		sw_param -> precall = 0;
	}

	return(1);
}

//~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int open_urg_sensor_ethernet(urg_t *urg)
{
#if defined(URG_WINDOWS_OS)
    const char *device = "COM3";
#elif defined(URG_LINUX_OS)
    const char *device = "/dev/ttyACM0";
    //const char *device = "/dev/ttyUSB0";
#else
    const char *device = "/dev/tty.usbmodemfa131";
#endif
    urg_connection_type_t connection_type = URG_SERIAL;
    long baudrate_or_port = 115200;
    const char *ip_address = "192.168.0.10";
    //const char *ip_address = "localhost";
    
    // \~japanese ê⁄ë±É^ÉCÉvÇÃêÿë÷Ç¶
    connection_type = URG_ETHERNET;
    baudrate_or_port = 10940;
    device = ip_address;
    
    // \~japanese ê⁄ë±
    if (urg_open(urg, connection_type, device, baudrate_or_port) < 0) {
        printf("urg_open: %s, %ld: %s\n",
               device, baudrate_or_port, urg_error(urg));
        return -1;
    }
    return 0;
}

int scan_in_cycle(urg_t *urg, people2D_engine ppl2D) {
    long *data;
    long max_distance;
    long min_distance;
    long time_stamp;
    int i;
    int n;
    
    urg_start_measurement(urg, URG_DISTANCE, 1, 0);
    data = (long *)malloc(urg_max_data_size(urg) * sizeof(data[0]));
    if (!data) {
        perror("urg_max_index()");
        return 1;
    }
    
    n = urg_get_distance(urg, data, &time_stamp);
    if (n < 0) {
        printf("urg_get_distance: %s\n", urg_error(urg));
        urg_close(urg);
        return 1;
    }
    
    urg_distance_min_max(urg, &min_distance, &max_distance);
    laserscan_data onescan(n);
    onescan.timestamp = (double)time_stamp;
    for (i = 0; i < n; ++i) {
        double distance = data[i]/10.0;
        double radian;
        double x;
        double y;
        
        if ((distance < min_distance/10.0) || (distance > max_distance/10.0)) {
            continue;
        }
        
        radian = urg_index2rad(urg, i);
        x = (double)(distance * cos(radian));
        y = (double)(distance * sin(radian));
        
        onescan.data.pts[i].x = x;
        onescan.data.pts[i].y = y;
        onescan.data.pts[i].label = 0;
    }
    urg_stop_measurement(urg);
    order_bytheta_incart(onescan.data.pts);
    ppl2D.add_onescan(onescan);
    
    const char * filename = "/Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/LRF/netparams.txt";
    neuralnet net(ppl2D.lfeatures, filename);
    
    std::vector<LSL_Point3D_container> clusters;
    ppl2D.segment(clusters);
    std::vector<LSL_Point3D_container> legs = net.classifyClusters(clusters);
    // Let's assume that we have acquired legs here. Have to pass them to other C++ code now
    
    free(data);
    #if defined(URG_MSC)
    getchar();
    #endif
    return 0;
}

people2D_engine setup_people_engile()
{
    sw_param_str sw_param;
    sw_param.sanity = 1;
	sw_param.segonly = 0;
	sw_param.verbosity = 0;
	sw_param.precall = 0;
	sw_param.benchmark = 0;
    
    sw_param.featuremix = 2;
    sw_param.dseg = 0.2;
    
    people2D_engine ppl2D(sw_param);
    ppl2D.set_featureset();
    return ppl2D;
}

int main (int argc, char **argv)
{
    urg_t urg;
    
    if (open_urg_sensor_ethernet(&urg) < 0) {
        printf("Couldn't establish connection to sensor");
        return 1;
    }
    
    people2D_engine ppl2D = setup_people_engile();
    
    while (true) {
        if (scan_in_cycle(&urg, ppl2D)) {
            break;
        }
        sleep(1.0f);
    }
    return 0;
    
    urg_close(&urg);
    
    /*int ret = ppl2D.load_scandata(sw_param.inputfile);
    if(!ret)
    {
        printf("No data or error in parsing\n");
        exit(1);
    }
    printf("File contains %d laser scans\n", ret);
  
    const char * filename = "/Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/LRF/netparams.txt";
    neuralnet net(ppl2D.lfeatures, filename);
  
    std::vector<LSL_Point3D_container> clusters;
    ppl2D.segment(clusters);
    std::vector<LSL_Point3D_container> legs = net.classifyClusters(clusters);*/
    
    // Let's assume that we have acquired legs here. Have to pass them to other C++ code now
  
  return 0;
  
  
	/*sw_param_str sw_param;
	
	if(!parse_command_line(argc, argv, &sw_param))
		exit(0);
	printf("Attempting to load file: [%s]\n", sw_param.inputfile.c_str());
	people2D_engine ppl2D(sw_param);
	int ret = ppl2D.load_scandata(sw_param.inputfile);

	if(!ret)
	{
		printf("No data or error in parsing\n");
		exit(1);
	}
	printf("File contains %d laser scans\n", ret);
	
	if(sw_param.segonly)
	{
		printf("Segmentation only dumping in segments.seg\n");
		ppl2D.segment_and_save("segments.seg");	
	}
	else
	{
		printf("Setting up\n");
		ppl2D.set_featureset();
		printf("Load model\n");
		int reta = ppl2D.load_adaboost_model(sw_param.modelfile);
		if(!reta)
		{
			printf("Cannot open [%s]\n",sw_param.modelfile.c_str());
			exit(1);
		}
		struct timeval start_clk, stop_clk;
		gettimeofday(&start_clk, (struct timezone *)NULL);
	
		if(!sw_param.benchmark)
			printf("Detect and save..");
		else	
			printf("Detect only..");
		fflush(stdout);
		if(sw_param.precall)
			printf("and precision-recall computation..");fflush(stdout);
		ppl2D.detect_save_all();

		gettimeofday(&stop_clk, (struct timezone *)NULL);
		double t1 =  (double)start_clk.tv_sec + (double)start_clk.tv_usec/(1000000);
		double t2 =  (double)stop_clk.tv_sec + (double)stop_clk.tv_usec/(1000000);
		double elapsed = t2 - t1;		
		
		printf("completed in %g s [avg per scan %g Hz]\n",elapsed, (double)ret/elapsed);    
		
		if(sw_param.precall)
		{	
			char filen[800];
			sprintf(filen, "%s.prec",sw_param.outputfile.c_str());

			printf("Saving precision-recall file in '%s' -- use matlab script (or other) to visualize it\n",filen);
			ppl2D.save_precall(filen);
		}
	}*/
  
  
}
