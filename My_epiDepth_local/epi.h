#include<opencv2/opencv.hpp>

#define EPI_WIDTH    /*4007*/  /*4007*/   /*1024*/       /*1280*/   	/*2622*/   1280
#define EPI_HEIGHT   /*101*/   /*101*/      /*17*/         /*17*/       /*151*/     17
#define EPI_NUM      /*2622*/  /*2622*/    /*1024*/      /*960*/       /*1718*/     960
#define DEPTH_WIDTH EPI_WIDTH 
#define DEPTH_HEIGHT EPI_NUM
#define CENTER_ROW (EPI_HEIGHT/2)
	
struct Pixel
{
	double b,g,r;
	double para;
};
struct Point
{
	int x,y;
	uchar d;
	double c;
};
class CEPI
{
public:
	void Init();
	void run();

protected:
	int getSupportPoint(uchar* depth,Point*,int);
	void openOperate(bool*);
	void getEdgeDepth(Point*,uchar*,double* depth_c,int,int);
	void depthPropagation(Point*,uchar* depth,double* depth_c,int);
	void getRangePara();
	int getSupportSet(Pixel*,uchar*,int pos,int height,int width,int widthStep,double perRange);
	double getDistance(Pixel,Pixel*,int,double);

private:
	IplImage* m_src[15];
	int m_epiNum;
	double m_step;
	int m_supNumber;
	uchar m_maxD;
	double m_sigma;
	double m_phy;
	double m_rangePara[EPI_HEIGHT/2];
	double t_sigma;
	int m_pyr;
	int m_midNum;//记录中间的number，为了分两半计算
	double m_crossPara;

};