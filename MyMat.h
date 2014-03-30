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
	//ת��
	MyMat transpose();		
	//����ʽ
	double det();
	//����
	bool inverse(MyMat &res);
	//ȡԪ��
	double &at(int i,int j);

	//QR�ֽ�
	bool QR(MyMat &Q,MyMat &R);

	//Ŀǰ�㷨ֻ�ܴ�����
	//svd����ֵ�ֽ�
	void SVD(MyMat &U,MyMat &S,MyMat &V);


	//�ݷ����������ֵ�Լ���Ӧ����������
	void powermethod(double &y,MyMat &Y);

	//��ӡ
	void show();
	//����
	static void test();
private:
	//��������ֵ
	void swap(double *a,double *b);
	//��������
	void swap(double *a,double *b,int n);
	//��a�е�ÿһ��Ԫ��*k �ӵ�b��ÿһ��Ԫ����
	void lineOper(double *a,double *b,double k,int n);
};
