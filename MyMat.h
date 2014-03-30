#pragma once

class MyMat
{
private :
	int rows;
	int cols;
	double *data;
public:
	MyMat(int rows,int cols);
	MyMat(int rows,int cols,double *tem);
	~MyMat(void);
	MyMat(const MyMat &b);
	MyMat &operator=(const MyMat &b);
	MyMat operator*(MyMat &b);
	MyMat operator*(double b);
	MyMat &operator*=(double b);
	MyMat operator+(MyMat &b);
	MyMat operator+(double b);
	MyMat &operator+=(MyMat &b);
	MyMat &operator+=(double b);
	MyMat operator-(MyMat &b);
	MyMat operator-(double b);
	MyMat &operator-=(MyMat &b);
	MyMat &operator-=(double b);
	//转置
	MyMat transpose();		
	//行列式
	double det();
	//求逆
	bool inverse(MyMat &res);
	//取元素
	double &at(int i,int j);

	//QR分解
	bool QR(MyMat &Q,MyMat &R);

	//目前算法只能处理方阵
	//svd奇异值分解
	void SVD(MyMat &U,MyMat &S,MyMat &V);


	//幂法求最大特征值以及对应的特征向量
	void powermethod(double &y,MyMat &Y);

	//打印
	void show();
	//测试
	static void test();
private:
	//交换两个值
	void swap(double *a,double *b);
	//交换两行
	void swap(double *a,double *b,int n);
	//将a行的每一个元素*k 加到b的每一个元素上
	void lineOper(double *a,double *b,double k,int n);
};
