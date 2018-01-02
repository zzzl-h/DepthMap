#include <iostream>
#include"epi.h"
#include<time.h>
using namespace std;  
using namespace cv;     

string path = "D:\\epi_car\\";

IplImage* depthImage = cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);
int main()
{
	time_t begin,end;
	double cost;
	begin = clock();
	CEPI epi;
	epi.Init();
	epi.run();   
	end = clock();
	cost = (double)(end-begin)/1000;
	printf("total time is %f s\n",cost);
	system("pause");
	return 0;
}


