#include "basic.h"
#include<stdio.h>
#include<string.h>
#include <time.h>
#include<fstream>
extern std::string path;	
extern IplImage* depthImage;

void CEPI::Init()
{
	m_step = 13.0/255.0;                             //
	m_maxD = 255;
	m_pyr = 1;
	m_sigma = 1.0/255.0;
	t_sigma = 4.0/255.0;
	m_crossPara = 1;
	m_midNum = 0;
	m_phy = 0.00000005;
}

void CEPI::run()
{
	memset(m_rangePara,0,sizeof(double)*(EPI_HEIGHT/2));
	//getRangePara();
	for (int num=0;num<1;num++)
	{    
		time_t begin,end;
		double cost;
		begin = clock();
		m_epiNum = num;
		char filename[255];
		char filename2[255];
		char filename3[255];
		int k = num;
		strcpy(filename,path.c_str());
		sprintf(filename+path.size(),"%d.jpg",k);
		strcpy(filename2, path.c_str());
		sprintf(filename2 + path.size(), "%d.jpg", k+1);
		strcpy(filename3, path.c_str());
		sprintf(filename3 + path.size(), "%d.jpg", 0);
		

		//Point supportPoint[EPI_WIDTH];
		uchar depth[EPI_WIDTH];
		double depth_c[EPI_WIDTH];
		memset(depth,0,EPI_WIDTH*sizeof(uchar));
		//for(int pyr=0;pyr<=m_pyr-1;pyr++)
		//{
			//if (0==pyr)
	    m_src[0] = cvLoadImage(filename);
	    if (num < EPI_NUM-1){
		  m_src[1] = cvLoadImage(filename2);
					}
	    else m_src[1] = cvLoadImage(filename3);

		//--------------------------------------------------------------------------------------------
		Point supportPoint[EPI_WIDTH];
		

			//Point supportPoint[EPI_WIDTH];

		///---------------getSupportPoint-----------------------------------------------------
		int supNumber = getSupportPoint(depth,supportPoint,0);

		///---------------getEdgeDepth-----------------------------------------------------
		getEdgeDepth(supportPoint,depth,depth_c,supNumber,0);

		depthPropagation(supportPoint,depth,depth_c,supNumber);

		

		//int supNumber = 0;
		//for (int i=0;i<EPI_WIDTH;i++)
		//{
		//	if(!depth[i])
		//		continue;
		//	supportPoint[supNumber].x = i;
		//	supportPoint[supNumber].d = depth[i];//////////////原来问题出现在这里，你妹呀！！！
		//	
		//	supNumber++;
		//}
		///--------------------depthPropagation----------------------------------------------------
		///depthPropagation(supportPoint,depth,depth_c,supNumber);

		for (int pyr=0;pyr<=m_pyr;pyr++)
		{
			cvReleaseImage(&m_src[pyr]);//
			m_src[pyr] = NULL;
			
		}
		int block = depthImage->widthStep*num;
		memcpy((uchar*)depthImage->imageData+block,depth,DEPTH_WIDTH);
		
		end = clock();
		cost = (double)(end-begin)/1000;
		printf("time for %d.jpg is %f s\n",num,cost);
	}
	//////////////////////////////////////////////////////////////////////////////////////
	cvSaveImage(/*"edge_m2_m2_noOpen.jpg"*/"car.jpg",depthImage);
	cvReleaseImage(&depthImage);
}


void CEPI::getEdgeDepth(Point* supportPoint,uchar* depth,double* depth_c,int supNumber,int pyr)
{
	int m = (int)pow(2.0,0);
	int height = m_src[0]->height;
	int width = m_src[0]->width;
	int center = height/2;
	int widthStep = m_src[0]->widthStep;
	int block = widthStep*center/*CENTER_ROW*/;
	uchar* data= (uchar*)m_src[0]->imageData;
	Pixel ref,sup[EPI_HEIGHT];
	for (int i=0;i<supNumber;i++)
	{
		int pos = supportPoint[i].x; 
		double energy=0.0,energy1=0.0,energy2=0.0,minEnergy= 900.0;
		ref.b = (uchar)data[block+3*supportPoint[i].x]/255.0;
		ref.g = (uchar)data[block+3*supportPoint[i].x+1]/255.0;
		ref.r = (uchar)data[block+3*supportPoint[i].x+2]/255.0;
		uchar slope_ = 0;
		uchar for_slope = 0;
		
		for (int slope=1;slope<m_maxD;slope+=4)           /////调slope的范围--必须从1开始
		{
			double perRangePos;
			perRangePos = m_step*slope;
			int setNumber1 = getSupportSet(sup,data,pos,height,width,widthStep,perRangePos);
			energy = getDistance(ref,sup,setNumber1,m_sigma);
			printf("%d ", &energy);
			if (energy<minEnergy)
			{
				minEnergy = energy;
				slope_ = slope;
			}
		}
		for_slope = slope_;
		for (int slope=for_slope-15;slope<for_slope+15;slope+=1)           /////调slope的范围--必须从1开始
		{
			if(slope<1||slope>m_maxD)
				continue;
			double perRangePos;
			perRangePos = m_step*slope;
			int setNumber1 = getSupportSet(sup,data,pos,height,width,widthStep,perRangePos);
			energy = getDistance(ref,sup,setNumber1,m_sigma);

			if (energy<minEnergy)
			{
				minEnergy = energy;
				slope_  =     slope;
				
			}
		}
		
		////for_slope = slope_;
		////for (int slope=for_slope-8;slope<for_slope+8;slope+=1)           /////调slope的范围--必须从1开始
		////{
		////	if(slope<1||slope>m_maxD)
		////		continue;
		////	double perRangePos;
		////	perRangePos = m_step*slope;
		////	int setNumber1 = getSupportSet(sup,data,pos,height,width,widthStep,perRangePos);
		////	energy = getDistance(ref,sup,setNumber1,m_sigma);
		////	if (energy<minEnergy)
		////	{
		////		minEnergy = energy;
		////		slope_  =     slope;
		////	}
		//}
		
		if(minEnergy<999.0)                                                              ///
		{
			depth[pos*m] = slope_;       ///---------------只有正向时----------------------------------
			supportPoint[i].d = slope_;
			///double score[2] = {1-minEnergy[0],1-minEnergy[1]};
			///supportPoint[i].c = 0.5*(score[0]-score[1])/score[0]+0.5*score[0];
		}
		
	}
}


void CEPI::depthPropagation(Point* supportPoint,uchar* depth,double* depth_c,int supNumber)
{
	int block = m_src[0]->widthStep*CENTER_ROW;
	int widthStep = m_src[0]->widthStep;
	uchar* data = (uchar*)m_src[0]->imageData;
	Pixel /*ref_l,ref_r,*/sup[EPI_HEIGHT+100];

	for (int i=0;i<supNumber-1;i++)
	{
		uchar min_d,max_d,d_left,d_right;
		int pos_left,pos_right;
		Pixel left,right;
		pos_left  = supportPoint[i].x;
		pos_right = supportPoint[i+1].x;
		if(1==pos_right-pos_left)
			continue;
		left.b  = data[block+pos_left*3]/255.0; left.g  = data[block+pos_left*3+1]/255.0;  left.r = data[block+pos_left*3+2]/255.0;
		right.b = data[block+pos_right*3]/255.0;right.g = data[block+pos_right*3+1]/255.0;right.r = data[block+pos_right*3+2]/255.0;
		d_left    = supportPoint[i].d;
		d_right   = supportPoint[i+1].d;
		
		//-----------------对所有中间的进行扩散 方法2 begin---------------------------------------------------------------
		for (int j=pos_left+1;j<pos_right;j++)
		{
			int pos = j;
			double energy=0.0,energy1=0.0,energy2=0.0,minEnergy = 999;

			Pixel cur/*,cur_l,cur_r*/;
			cur.b = data[block+3*j]/255.0;
			cur.g = data[block+3*j+1]/255.0;
			cur.r = data[block+3*j+2]/255.0;
			/*cur_l.b = data[block+3*j-3]/255.0;
			cur_l.g = data[block+3*j-2]/255.0;
			cur_l.r = data[block+3*j-1]/255.0;
			cur_r.b = data[block+3*j+3]/255.0;
			cur_r.g = data[block+3*j+4]/255.0;
			cur_r.r = data[block+3*j+5]/255.0;*/

			double t_sigma2 = 2*t_sigma*t_sigma;
			/*Pixel disp;
			disp.b = cur.b-cur_l.b;disp.g = cur.g-cur_l.g;disp.r = cur.r-cur_l.r;*/

			double left_pxl,right_pxl,disp_left_pxl,disp_right_pxl,disp_left_dist,disp_right_dist;
			left_pxl  = (abs(cur.b-left.b) +abs(cur.g-left.g)+ abs(cur.r-left.r))/3;
			right_pxl = (abs(cur.b-right.b)+abs(cur.g-right.g)+abs(cur.r-right.r))/3;
			disp_left_pxl  = left_pxl/(left_pxl+right_pxl);
			disp_right_pxl = right_pxl/(left_pxl+right_pxl);
			disp_left_dist = (pos-pos_left)/(pos_right-pos_left+0.00000000001);
			disp_right_dist= (pos_right-pos)/(pos_right-pos_left+0.00000000001);
			int ref_slope = (int)((0.5*disp_right_pxl+0.5*disp_right_dist)*d_left+(0.5*disp_left_pxl+0.5*disp_left_dist)*d_right+0.4999);
			for (int slope=ref_slope-5*3;slope<=ref_slope+5*3;slope++)   ///
			{
				///----------------------只有正向 begin-----------------------------------------
				if(slope<1 || slope>m_maxD)                //
					continue;
				//  更改disp_d
				uchar disp_d = abs(slope-depth[pos-1]/2);

				double perRange,prior,likehood;
				prior = -0.1*log(m_phy+exp(-(pow(slope-ref_slope,2.0)/(2*pow(24.0,2)))));
				perRange = m_step*slope;
				int setNumber = getSupportSet(sup,data,pos,EPI_HEIGHT,EPI_WIDTH,widthStep,perRange);
				likehood = getDistance(cur,sup,setNumber,m_sigma);
				energy = prior + 10*likehood;
				if (energy<minEnergy)
				{
					minEnergy = energy;
					depth[pos] = slope;                                //
					//supportPoint[pos].d = slope;
					//depth_c[pos] = minEnergy;
				}
				///----------------------只有正向 end-----------------------------------------

			}
		}

		//-----------------对所有中间的进行扩散 方法2 end---------------------------------------------------------------------
	}



	//---------------------------对左边缘点左边的进行扩散-----------------------------------------------
	for (int i=supportPoint[0].x-1;i>=0;i--)
	{
		int pos = i;
		double energy=0.0,energy1=0.0,energy2=0.0,minEnergy = 999.0;
		Pixel cur,cur_l;
		cur.b = data[block+3*i]/255.0;
		cur.g = data[block+3*i+1]/255.0;
		cur.r = data[block+3*i+2]/255.0;
		cur_l.b = data[block+3*i-3]/255.0;
		cur_l.g = data[block+3*i-2]/255.0;
		cur_l.r = data[block+3*i-1]/255.0;
		double t_sigma2 = 2*t_sigma*t_sigma;
		Pixel disp;
		disp.b = cur.b-cur_l.b;disp.g = cur.g-cur_l.g;disp.r = cur.r-cur_l.r;
		for (int slope=1;slope<supportPoint[0].d;slope++)
		{
			if(slope<1||slope>m_maxD)
				continue;                
			double perRange;
			perRange = m_step*slope;
			int setNumber = getSupportSet(sup,data,pos,EPI_HEIGHT,EPI_WIDTH,widthStep,perRange);
			energy = getDistance(cur,sup,setNumber,m_sigma);
			if (energy<minEnergy)
			{
				minEnergy = energy;
				depth[pos] = slope;                                   //
			}
		}
	}
	//---------------------------对右边缘点右边的进行扩散----------------------------------------
	for (int i=supportPoint[supNumber-1].x+1;i<EPI_WIDTH;i++)
	{
		int pos = i;
		double energy=0.0,energy1=0.0,energy2=0.0,minEnergy = 999;
		Pixel cur,cur_l;
		cur.b = data[block+3*i]/255.0;
		cur.g = data[block+3*i+1]/255.0;
		cur.r = data[block+3*i+2]/255.0;
		cur_l.b = data[block+3*i-3]/255.0;
		cur_l.g = data[block+3*i-2]/255.0;
		cur_l.r = data[block+3*i-1]/255.0;
		double t_sigma2 = 2*t_sigma*t_sigma;
		Pixel disp;
		disp.b = cur.b-cur_l.b;disp.g = cur.g-cur_l.g;disp.r = cur.r-cur_l.r;
		for (int slope=supportPoint[supNumber-1].d-5;slope<supportPoint[supNumber-1].d+5*3;slope++)
		{
			if(slope<1||slope>m_maxD)
				continue;
			//uchar disp_d = abs(slope-supportPoint[supNumber-1].d);
			uchar disp_d = abs(slope-depth[pos-1]/2/*supportPoint[supNumber-1].d*/);
			double perRange;
			perRange = m_step*slope;
			int setNumber = getSupportSet(sup,data,pos,EPI_HEIGHT,EPI_WIDTH,widthStep,perRange);
			energy = getDistance(cur,sup,setNumber,m_sigma);
			if (energy<minEnergy)
			{
				minEnergy = energy;
				depth[pos] = slope;                                 //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
				//supportPoint[pos].d = slope;
				//depth_c[pos] = minEnergy;
			}
		}
	}
}

/*------------------------------------------------------------------------------------------\
getSupportPoint()
\------------------------------------------------------------------------------------------*/
int CEPI::getSupportPoint(uchar* depth,Point* supportPoint,int pyr)
{
	/*int supNum = 0;
	for(int ii=0;ii<EPI_WIDTH;ii++)
	{
		supportPoint[supNum].x = ii;
		supNum++;

	}
	return supNum;*/
	int crossMask = 9;
	int m = (int)pow(2.0,0);
	int supNum = 0;
	////////////////////////////////////////////////////////////
	int width = m_src[0]->width;	
	int center = m_src[0]->height/2;
	int step = m_src[0]->widthStep;
	uchar* data = (uchar*)m_src[0]->imageData;
	int block = step*center;//
	//////////////////////////////////////////////////////////////
	int width2 = m_src[1]->width;
	int center2 = m_src[1]->height / 2;
	int step2 = m_src[1]->widthStep;
	uchar* data2 = (uchar*)m_src[1]->imageData;
	int block2= step2*center2;//

	bool tmpTabel[EPI_WIDTH+1000];
	memset(tmpTabel,0,sizeof(bool)*EPI_WIDTH);//赋值0

	bool tmpTabel2[EPI_WIDTH+1000];
	memset(tmpTabel2, 0, sizeof(bool)*EPI_WIDTH);

	bool tmpTabel3[EPI_WIDTH + 1000];
	memset(tmpTabel3, 0, sizeof(bool)*EPI_WIDTH);

	int width3 = m_src[0]->width;
	int center3 = m_src[0]->height / 2 +1;
	int step3 = m_src[0]->widthStep;
	uchar* data3 = (uchar*)m_src[0]->imageData;
	int block3 = step3*center3;//

	Pixel ref,cur;
	
	for(int i=5;i<width-5;i++)
	{
		if(depth[m])
			continue;
		int block_ref = block+3*i;
		//像素归一化
		ref.b = data[block_ref]/255.0;
		ref.g = data[block_ref+1]/255.0;
		ref.r = data[block_ref+2]/255.0;

		
		
		///--------------=using cross detector to detect edges-----------------
		/*-------------------------------method 1 begin------------------------------------------------*/
		/*double sum1=0.0,sum2=0.0;
		///horizontal
		for(int j=-crossMask/2;j<=crossMask/2;j++)
		{
			cur.b = data[block_ref+3*j]/255.0;
			cur.g = data[block_ref+3*j+1]/255.0;
			cur.r = data[block_ref+3*j+2]/255.0;

			sum1 += (abs(ref.b-cur.b)+abs(ref.g-cur.g)+abs(ref.r-cur.r))/3;
		}
		sum1 /= (crossMask-1);
		///vertical
		for(int k=-crossMask/2;k<=crossMask/2;k++)
		{
			cur.b = data[block_ref+step*k]/255.0;
			cur.g = data[block_ref+step*k+1]/255.0;
			cur.r = data[block_ref+step*k+2]/255.0;

			sum2 += (abs(ref.b-cur.b)+abs(ref.g-cur.g)+abs(ref.r-cur.r))/3;
		}
		sum2/=(crossMask-1);*/
		/*-------------------------------method 1 end------------------------------------------------*/

		/*-------------------------------method 2 begin------------------------------------------------*/
		double sum1_b=0.0,sum1_g=0.0,sum1_r=0.0,sum2_b=0.0,sum2_g=0.0,sum2_r=0.0;
		double sum1=0.0,sum2=0.0;
		///horizontal
		for(int j=-crossMask/2;j<=crossMask/2;j++)
		{
			sum1_b += pow(data[block_ref+3*j]/255.0-ref.b,2);
			sum1_g += pow(data[block_ref+3*j+1]/255.0-ref.g,2);
			sum1_r += pow(data[block_ref+3*j+2]/255.0-ref.r,2);
			/*sum2_b += data[block_ref-3*j]/255.0;
			sum2_g += data[block_ref-3*j+1]/255.0;
			sum2_r += data[block_ref-3*j+2]/255.0;*/
		}
		sum1 = (sum1_b+sum1_g+sum1_r)/3;
		///vertical
		for(int j=-crossMask/2;j<=crossMask/2;j++)
		{
			/*sum1_b += data[block_ref+step*j]/255.0;
			sum1_g += data[block_ref+step*j+1]/255.0;
			sum1_r += data[block_ref+step*j+2]/255.0;*/
			sum2_b += pow(data[block_ref+step*j]/255.0-ref.b,2);
			sum2_g += pow(data[block_ref+step*j+1]/255.0-ref.g,2);
			sum2_r += pow(data[block_ref+step*j+2]/255.0-ref.r,2);
		}
		sum2 = (sum2_b+sum2_g+sum2_r)/3;
		/*-------------------------------method 2 end------------------------------------------------*/


		if((sum1+sum2)>m_crossPara)
		//if(sum1>m_crossPara)
		{
			//supportPoint[supNum].x = i;
			//supportPoint[supNum].y = center;
			tmpTabel[i] = true;
			//supNum++;
		}
	}
	Pixel ref2, cur2;
	for (int i = 5; i<width2 - 5; i++)
	{
		if (depth[m])
			continue;
		int block_ref2 = block2 + 3 * i;
		//像素归一化
		ref2.b = data2[block_ref2] / 255.0;
		ref2.g = data2[block_ref2 + 1] / 255.0;
		ref2.r = data2[block_ref2 + 2] / 255.0;



		///--------------=using cross detector to detect edges-----------------
		/*-------------------------------method 1 begin------------------------------------------------*/
		/*double sum1=0.0,sum2=0.0;
		///horizontal
		for(int j=-crossMask/2;j<=crossMask/2;j++)
		{
		cur.b = data[block_ref+3*j]/255.0;
		cur.g = data[block_ref+3*j+1]/255.0;
		cur.r = data[block_ref+3*j+2]/255.0;

		sum1 += (abs(ref.b-cur.b)+abs(ref.g-cur.g)+abs(ref.r-cur.r))/3;
		}
		sum1 /= (crossMask-1);
		///vertical
		for(int k=-crossMask/2;k<=crossMask/2;k++)
		{
		cur.b = data[block_ref+step*k]/255.0;
		cur.g = data[block_ref+step*k+1]/255.0;
		cur.r = data[block_ref+step*k+2]/255.0;

		sum2 += (abs(ref.b-cur.b)+abs(ref.g-cur.g)+abs(ref.r-cur.r))/3;
		}
		sum2/=(crossMask-1);*/
		/*-------------------------------method 1 end------------------------------------------------*/

		/*-------------------------------method 2 begin------------------------------------------------*/
		double sum12_b = 0.0, sum12_g = 0.0, sum12_r = 0.0, sum22_b = 0.0, sum22_g = 0.0, sum22_r = 0.0;
		double sum12 = 0.0, sum22 = 0.0;
		///horizontal
		for (int j = -crossMask / 2; j <= crossMask / 2; j++)
		{
			sum12_b += pow(data2[block_ref2 + 3 * j] / 255.0 - ref2.b, 2);
			sum12_g += pow(data2[block_ref2+ 3 * j + 1] / 255.0 - ref2.g, 2);
			sum12_r += pow(data2[block_ref2 + 3 * j + 2] / 255.0 - ref2.r, 2);
			/*sum2_b += data[block_ref-3*j]/255.0;
			sum2_g += data[block_ref-3*j+1]/255.0;
			sum2_r += data[block_ref-3*j+2]/255.0;*/
		}
		sum12 = (sum12_b + sum12_g + sum12_r) / 3;
		///vertical
		for (int j = -crossMask / 2; j <= crossMask / 2; j++)
		{
			/*sum1_b += data[block_ref+step*j]/255.0;
			sum1_g += data[block_ref+step*j+1]/255.0;
			sum1_r += data[block_ref+step*j+2]/255.0;*/
			sum22_b += pow(data2[block_ref2 + step*j] / 255.0 - ref2.b, 2);
			sum22_g += pow(data2[block_ref2 + step*j + 1] / 255.0 - ref2.g, 2);
			sum22_r += pow(data2[block_ref2 + step*j + 2] / 255.0 - ref2.r, 2);
		}
		sum22 = (sum22_b + sum22_g + sum22_r) / 3;
		/*-------------------------------method 2 end------------------------------------------------*/


		if ((sum12 + sum22)>m_crossPara)
			//if(sum1>m_crossPara)
		{
			//supportPoint[supNum].x = i;
			//supportPoint[supNum].y = center;
			tmpTabel2[i] = true;
			//supNum++;
		}
	}

	Pixel ref3, cur3;
	for (int i = 5; i<width2 - 5; i++)
	{
		if (depth[m])
			continue;
		int block_ref3 = block3 + 3 * i;
		//像素归一化
		ref3.b = data2[block_ref3] / 255.0;
		ref3.g = data2[block_ref3 + 1] / 255.0;
		ref3.r = data2[block_ref3 + 2] / 255.0;



		///--------------=using cross detector to detect edges-----------------
		/*-------------------------------method 1 begin------------------------------------------------*/
		/*double sum1=0.0,sum2=0.0;
		///horizontal
		for(int j=-crossMask/2;j<=crossMask/2;j++)
		{
		cur.b = data[block_ref+3*j]/255.0;
		cur.g = data[block_ref+3*j+1]/255.0;
		cur.r = data[block_ref+3*j+2]/255.0;

		sum1 += (abs(ref.b-cur.b)+abs(ref.g-cur.g)+abs(ref.r-cur.r))/3;
		}
		sum1 /= (crossMask-1);
		///vertical
		for(int k=-crossMask/2;k<=crossMask/2;k++)
		{
		cur.b = data[block_ref+step*k]/255.0;
		cur.g = data[block_ref+step*k+1]/255.0;
		cur.r = data[block_ref+step*k+2]/255.0;

		sum2 += (abs(ref.b-cur.b)+abs(ref.g-cur.g)+abs(ref.r-cur.r))/3;
		}
		sum2/=(crossMask-1);*/
		/*-------------------------------method 1 end------------------------------------------------*/

		/*-------------------------------method 2 begin------------------------------------------------*/
		double sum13_b = 0.0, sum13_g = 0.0, sum13_r = 0.0, sum23_b = 0.0, sum23_g = 0.0, sum23_r = 0.0;
		double sum13 = 0.0, sum23 = 0.0;
		///horizontal
		for (int j = -crossMask / 2; j <= crossMask / 2; j++)
		{
			sum13_b += pow(data3[block_ref3 + 3 * j] / 255.0 - ref3.b, 2);
			sum13_g += pow(data3[block_ref3 + 3 * j + 1] / 255.0 - ref3.g, 2);
			sum13_r += pow(data3[block_ref3 + 3 * j + 2] / 255.0 - ref3.r, 2);
			/*sum2_b += data[block_ref-3*j]/255.0;
			sum2_g += data[block_ref-3*j+1]/255.0;
			sum2_r += data[block_ref-3*j+2]/255.0;*/
		}
		sum13 = (sum13_b + sum13_g + sum13_r) / 3;
		///vertical
		for (int j = -crossMask / 2; j <= crossMask / 2; j++)
		{
			/*sum1_b += data[block_ref+step*j]/255.0;
			sum1_g += data[block_ref+step*j+1]/255.0;
			sum1_r += data[block_ref+step*j+2]/255.0;*/
			sum23_b += pow(data3[block_ref3 + step*j] / 255.0 - ref2.b, 2);
			sum23_g += pow(data3[block_ref3 + step*j + 1] / 255.0 - ref2.g, 2);
			sum23_r += pow(data3[block_ref3 + step*j + 2] / 255.0 - ref2.r, 2);
		}
		sum23 = (sum23_b + sum23_g + sum23_r) / 3;
		/*-------------------------------method 2 end------------------------------------------------*/


		if ((sum13 + sum23)>m_crossPara)
			//if(sum1>m_crossPara)
		{
			//supportPoint[supNum].x = i;
			//supportPoint[supNum].y = center;
			tmpTabel3[i] = true;
			//supNum++;
		}
	}


	int flag = 0;
	for (int k = 2; k <EPI_WIDTH-2; k++){
	   if (tmpTabel[k] = true){
	     for (int s= k - 3; s < k + 3; s++)
		 {
	          if (tmpTabel2[k] = true)
	             flag = 1;
	               }
	        } 
	  if (flag = 0)
		   tmpTabel[k] = false;
	}


	

	///------------------------------ open operate-----------------------------
	openOperate(tmpTabel);
	for(int ii=0;ii<EPI_WIDTH;ii++)
	{
		if(tmpTabel[ii])
		{
			supportPoint[supNum].x = ii;
			supNum++;

			//////////////////测试边缘用////////////////////
			//depth[ii] = 255;
			
		}
	}
	return supNum;
}



void CEPI::openOperate(bool* LineSet)
{
	int width = m_src[0]->width;
	int height = m_src[0]->height;
	int center =  height/2;
	 //腐蚀
	for (int i=1;i<width;i++)
	{
		if(LineSet[i])
			if (!LineSet[i-1] && !LineSet[i+1])
			{
				LineSet[i] = false;
			}
	}
	//膨胀
	for (int i=1;i<width;i++)
	{
		if (!LineSet[i])
			if (LineSet[i-1] && LineSet[i+1])
			{
				LineSet[i] = true;	
			}
	}
}



int CEPI::getSupportSet(Pixel* supSet,uchar* data,int pos,int height,int width,int widthStep,double perRange)
{
	
	Pixel cur;
	int ref_x,ref_y;
	double cur_x,range;int cur_y;
	int left_x,right_x;
	ref_x = pos;   ref_y = height/2/*CENTER_ROW*/;
	int number = 0;
	for (int i=0/*height*/;i<height;i++)
	{
		if(i==EPI_HEIGHT/2)
			m_midNum = number;
		cur_y = i;
		cur_x = ref_x + (ref_y-cur_y)*perRange;
		if(cur_x<0 || cur_x>= /*EPI_WIDTH*/width)
			continue;
		int disRange = abs(EPI_HEIGHT/2-i);
		left_x = floor(cur_x);right_x = left_x+1;
		range = cur_x-left_x;
		cur.b = (range*data[cur_y*widthStep+3*right_x]  +(1-range)*data[cur_y*widthStep+3*left_x])/255.0;
		cur.g = (range*data[cur_y*widthStep+3*right_x+1]+(1-range)*data[cur_y*widthStep+3*left_x+1])/255.0;
		cur.r = (range*data[cur_y*widthStep+3*right_x+2]+(1-range)*data[cur_y*widthStep+3*left_x+2])/255.0;
		cur.para = m_rangePara[disRange];
		supSet[number++] = cur;
		//number++;
	}
	return number;
}

double CEPI::getDistance(Pixel cur,Pixel* supSet,int number,double m_sigma)
{   
	double energy1=0.0,energy2=0.0;                                 ///
	double b = cur.b,g = cur.g,r = cur.r;
	Pixel sum;
	Pixel a;
	//double energy=0.0;
	if (m_midNum/(number)>0.6)
	{
		for (int i=0;i<m_midNum;i++)
		{
			/*sum.b = (supSet[i].b-b)*(supSet[i].b-b);
			sum.g = (supSet[i].g-g)*(supSet[i].g-g);
			sum.r = (supSet[i].r-r)*(supSet[i].r-r);*/
			Pixel tmp;
			tmp.b = (supSet[i].b - b) ;
			tmp.g = (supSet[i].g - g);
			tmp.r = (supSet[i].r - r);
			double para = supSet[i].para;
			double sigma_sigma = 2*pow(m_sigma,2);
			a.b = 1 / (1 + (supSet[i].b - b)*(supSet[i].b - b));
			a.g = 1 / (1 + (supSet[i].g - g)*(supSet[i].g - g));
			a.b = 1 / (1 + (supSet[i].r -g)*(supSet[i].r - r));
			//sum.b = 1 - 1 / (1 + tmp.b)* exp(-tmp.b*tmp.b / sigma_sigma);
			//sum.g = 1 - 1 / (1 + tmp.g)*exp(-tmp.g*tmp.g / sigma_sigma);
			//sum.r = 1 - 1 / (1 + tmp.r)* exp(-tmp.r*tmp.r / sigma_sigma);
			sum.b = (1 - exp(-tmp.b*tmp.b / sigma_sigma)) - 1.5* supSet[i].b*log2(supSet[i].b);
			sum.g = (1 - exp(-tmp.g*tmp.g / sigma_sigma)) - 1.5* supSet[i].g*log2(supSet[i].g);
			sum.r = (1 - exp(-tmp.r*tmp.r / sigma_sigma)) - 1.5* supSet[i].r*log2(supSet[i].r);
			energy1 += (sum.b+sum.g+sum.r)/3;
	

		}
		return energy1/m_midNum;
	}
	if (m_midNum/(number)<0.4)
	{
		for (int i=m_midNum;i<number;i++)
		{
			/*sum.b = (supSet[i].b-b)*(supSet[i].b-b);
			sum.g = (supSet[i].g-g)*(supSet[i].g-g);
			sum.r = (supSet[i].r-r)*(supSet[i].r-r);*/
			Pixel tmp;
			tmp.b = (supSet[i].b - b);
			tmp.g = (supSet[i].g - g);
			tmp.r = (supSet[i].r - r);
			double para = supSet[i].para;
			double sigma_sigma = 2*m_sigma*m_sigma;
			a.b = 1 / (1 + (supSet[i].b - b)*(supSet[i].b - b));
			a.g = 1 / (1 + (supSet[i].g - g)*(supSet[i].g - g));
			a.b = 1 / (1 + (supSet[i].r - g)*(supSet[i].r - r));
			//sum.b = 1 - 1 / (1 + tmp.b)* exp(-tmp.b*tmp.b / sigma_sigma);
			//sum.g = 1 - 1 / (1 + tmp.g)*exp(-tmp.g*tmp.g / sigma_sigma);
			//sum.r = 1 - 1 / (1 + tmp.r)* exp(-tmp.r*tmp.r / sigma_sigma);
			sum.b = (1 - exp(-tmp.b*tmp.b / sigma_sigma)) - 1.5* supSet[i].b*log2(supSet[i].b);
			sum.g = (1 - exp(-tmp.g*tmp.g / sigma_sigma)) - 1.5* supSet[i].g*log2(supSet[i].g);
			sum.r = (1 - exp(-tmp.r*tmp.r / sigma_sigma)) - 1.5* supSet[i].r*log2(supSet[i].r);
			energy2 += /*para**/(sum.b+sum.g+sum.r)/3;
		}
			return energy2/(number-m_midNum);
	}

	for (int i=0;i<m_midNum;i++)
	{
		/*sum.b = (supSet[i].b-b)*(supSet[i].b-b);
		sum.g = (supSet[i].g-g)*(supSet[i].g-g);
		sum.r = (supSet[i].r-r)*(supSet[i].r-r);*/
		Pixel tmp;
		tmp.b = (supSet[i].b - b);
		tmp.g = (supSet[i].g - g);
		tmp.r = (supSet[i].r - r);
		double para = supSet[i].para;
		double sigma_sigma = 2*pow(m_sigma,2);
		a.b = 1 / (1 + (supSet[i].b - b)*(supSet[i].b - b));
		a.g = 1 / (1 + (supSet[i].g - g)*(supSet[i].g - g));
		a.b = 1 / (1 + (supSet[i].r - g)*(supSet[i].r - r));
		//sum.b = 1 - 1 / (1 + tmp.b)* exp(-tmp.b*tmp.b / sigma_sigma);
		//sum.g = 1 - 1 / (1 + tmp.g)*exp(-tmp.g*tmp.g / sigma_sigma);
		//sum.r = 1 - 1 / (1 + tmp.r)* exp(-tmp.r*tmp.r / sigma_sigma);
		sum.b = (1 - exp(-tmp.b*tmp.b / sigma_sigma)) - 1.5* supSet[i].b*log2(supSet[i].b);
		sum.g = (1 - exp(-tmp.g*tmp.g / sigma_sigma)) - 1.5* supSet[i].g*log2(supSet[i].g);
		sum.r = (1 - exp(-tmp.r*tmp.r / sigma_sigma)) - 1.5* supSet[i].r*log2(supSet[i].r);
		energy1 += /*para**/(sum.b+sum.g+sum.r)/3;
	}

	for (int i=m_midNum;i<number;i++)
	{
		/*sum.b = (supSet[i].b-b)*(supSet[i].b-b);
		sum.g = (supSet[i].g-g)*(supSet[i].g-g);
		sum.r = (supSet[i].r-r)*(supSet[i].r-r);*/
		Pixel tmp;
		tmp.b = (supSet[i].b - b);
		tmp.g = (supSet[i].g - g);
		tmp.r = (supSet[i].r - r);
		double para = supSet[i].para;
		double sigma_sigma = 2*m_sigma*m_sigma;
		a.b = 1 / (1 + (supSet[i].b - b)*(supSet[i].b - b));
		a.g = 1 / (1 + (supSet[i].g - g)*(supSet[i].g - g));
		a.b = 1 / (1 + (supSet[i].r - g)*(supSet[i].r - r));
		//sum.b = 1 - 1 / (1 + tmp.b)* exp(-tmp.b*tmp.b / sigma_sigma);
		//sum.g = 1 - 1 / (1 + tmp.g)*exp(-tmp.g*tmp.g / sigma_sigma);
		//sum.r = 1 - 1 / (1 + tmp.r)* exp(-tmp.r*tmp.r / sigma_sigma);

		sum.b =  (1 - exp(-tmp.b*tmp.b / sigma_sigma)) - 1.5* supSet[i].b*log2(supSet[i].b);
		sum.g =  (1 - exp(-tmp.g*tmp.g / sigma_sigma)) - 1.5* supSet[i].g*log2(supSet[i].g);
		sum.r =  (1 - exp(-tmp.r*tmp.r / sigma_sigma)) - 1.5* supSet[i].r*log2(supSet[i].r);
		energy2 += /*para**/(sum.b+sum.g+sum.r)/3;
	}
	energy1/=m_midNum;
	energy2/=(number-m_midNum);
	if (energy1/energy2>0.8 && energy1/energy2<1/0.8)
		return (energy1*m_midNum+energy2*(number-m_midNum))/number;


	return energy1<energy2?energy1:energy2;
}

void CEPI::getRangePara()
{
	int range = EPI_HEIGHT/2;
	for(int i=0;i<range;i++)
	{
		m_rangePara[i] = (range-i)*(range-i)/(range*range+0.001);
	}
}