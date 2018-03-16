#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
using namespace std;
#include <stack>
#include <queue>
#include <math.h>
Mat visit(400,640,CV_8UC1,Scalar(0));
typedef struct
	{
		Point parent;
		double f,g,h;
	}data;
typedef struct 
	{
		Point p;
		double f;
	}node;
bool operator<(const node& a,const node& b)
{
	return a.f>b.f;
}
class PathPlanner
{
private:
	data values[640][400];
	Mat a;
	Point src;
	Point dest;
public:
	PathPlanner()
	{
		a=imread("a.png",1);
		src=findpt(a,1);
		dest=findpt(a,2);
		astar(src,dest,a);
	}
	bool iswhite(int m,int n,Mat a);
	bool isblocked(int i,int j,Mat a);
	void astar(Point start,Point end,Mat a);
	Point findpt(Mat a,int col);
	void tracepath(Point dest,Mat a);
	bool isblack (int m,int n,Mat a);
	double Hval(int row, int col, Point dest);
	bool isvalid(int i,int j,Mat b);
};
bool PathPlanner::iswhite (int m,int n,Mat a)
{
	if ((a.at<Vec3b>(m,n)[0]==255)&&(a.at<Vec3b>(m,n)[0]==255)&&(a.at<Vec3b>(m,n)[0]==255))
		return true;
	else 
		return false;
}
bool PathPlanner::isblocked (int i,int j,Mat a)
{
	int m,n;
	for (m=i-5;m<=i+5;m++)
	{
		for (n=j-10;n<=j+10;n++)
		{
			if (isvalid(m,n,a))
			{
			if (iswhite(m,n,a))
				return true;
			}
		}
	}
	return false;
}
bool PathPlanner::isvalid(int i,int j,Mat b)
{
	return ((i >= 0) && (i < b.rows) &&
           (j >= 0) && (j < b.cols));
}
double PathPlanner::Hval(int row, int col, Point dest)
{
	// return ((double)sqrt((row-dest.x)*(row-dest.x)
     //                       + (col-dest.y)*(col-dest.y)));
	int dx,dy;
	dx = abs(row - dest.x);
    dy = abs(col - dest.y);
    return ((double)(10*(dx + dy) + (1.4-2*10) * min(dx, dy)));
}
bool PathPlanner::isblack (int m,int n,Mat a)
{
	if ((a.at<Vec3b>(m,n)[0]==0)&&(a.at<Vec3b>(m,n)[0]==0)&&(a.at<Vec3b>(m,n)[0]==0))
		return true;
	else 
		return false;
}
void PathPlanner::tracepath(Point dest,Mat a)
{
	int col = dest.x;
    int row = dest.y;
 
    stack <Point> Path;
 
    while (!(values[col][row].parent.x == col
             && values[col][row].parent.y == row))
    {
        Path.push (Point(col,row));
        int temp_col = values[col][row].parent.x;
        int temp_row = values[col][row].parent.y;
        row = temp_row;
        col = temp_col;
    }
    Mat d=a.clone();
    Mat c;
    Path.push (Point(col,row));
    while (!Path.empty())
    {	
    	namedWindow("path",WINDOW_NORMAL);
        Point p = Path.top();
        cout<<p.x<<" "<<p.y<<endl;
        d.at<Vec3b>(p.y,p.x)[0]=255;
        imshow("path",d);
        waitKey(20);
        c=a.clone();
		rectangle(c,Rect(p.x-10,p.y-5,20,10),Scalar(255,255,255),CV_FILLED,8,0);
		namedWindow("moving car",WINDOW_NORMAL);
		imshow("moving car",c);
		waitKey(20);
		Path.pop();
    }
    waitKey(0);
}
Point PathPlanner::findpt(Mat a,int col)
{
	int sumx=0;
	int sumy=0;
	int count=0;
	int i,j;
	int c,d;
	if (col==1)
	{
		c=0;
		d=2;
	}
	else if(col==2)
	{
		c=0;
		d=1;
	}

	for (i=0;i<a.rows;i++)
	{
		for (j=0;j<a.cols;j++)
		{
			if (a.at<Vec3b>(i,j)[col]==255&&a.at<Vec3b>(i,j)[c]==0&&a.at<Vec3b>(i,j)[d]==0)
			{
				sumy=sumy+i;
				sumx=sumx+j;
				count++;
			}
		}
	}
	Point s;
	s.x=sumx/count;
	s.y=sumy/count;
	return (s);
}
void PathPlanner::astar(Point start,Point end,Mat a)
{
	int i,j;
	for (i=0;i<a.rows;i++)
	{
		for (j=0;j<a.cols;j++)
		{
			values[j][i].f=FLT_MAX;
			values[j][i].g=FLT_MAX;
			values[j][i].h=FLT_MAX;
			values[j][i].parent.x = -1;
            values[j][i].parent.y = -1;
		}
	}
	j=start.x;
	i=start.y;
	values[j][i].f=0;
	values[j][i].g=0;
	values[j][i].h=0;
	values[j][i].parent.x = j;
    values[j][i].parent.y = i;
    priority_queue <node> list;
    node s;
 	s.f=0.0;
 	s.p.x=j;
 	s.p.y=i;
 	list.push(s);
 	while(list.size()!=0)
 	{
 		node temp;
 		temp=list.top();
 		j=temp.p.x;
 		i=temp.p.y;
 		list.pop();
 		visit.at<uchar>(i,j)=255;
 		double gNew, hNew, fNew;
 		int m,n;
 		for (m=i-1;m<=i+1;m++)
 		{
 			for (n=j-1;n<=j+1;n++)
 			{
 				if (isvalid(m,n,a))
 				{
 					if (m==end.y&&n==end.x)
 					{
 						values[n][m].parent.x=j;
 						values[n][m].parent.y=i;
 						tracepath(end,a);
 						return;
 					}
 					else if ((visit.at<uchar>(m,n)==0)&&isblack(m,n,a)&&(!isblocked(m,n,a)))
 					{
 						if ((m==i-1&&n==j-1)||(m==i-1&&n==j+1)||(m==i+1&&n==j-1)||(m==i+1&&n==j+1))
		                gNew = values[j][i].g + 1.4;
		            	else
		            	gNew = values[j][i].g + 1.0;
		               hNew = Hval(m,n,end);
		               fNew = gNew + hNew;
		                 if (values[n][m].f==FLT_MAX||values[n][m].f>fNew)
		                 {
		                 	node a;
		                    a.p.y=m;
		                    a.p.x=n;
		                    a.f=fNew;
		                    list.push(a);
		                    values[n][m].f=fNew;
		                    values[n][m].g=gNew;
		                  	values[n][m].h=hNew;
		                    values[n][m].parent.x=j;
		                    values[n][m].parent.y=i;
		                 }
 					}
 			}
 		}
 	}

}
}
int main()
{
	PathPlanner obj1;
}