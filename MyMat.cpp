#include "MyMat.h"
#include <stdio.h>
#include <math.h>
#include "stdafx.h"


#define ZERO 0.0000005
//#define ZERO 0.000000000001
//#define ZERO 0.0000000000000000000000000000001

MyMat::MyMat(int rows,int cols)
{
	this->rows = rows;
	this->cols = cols;
	this->data = new double[rows*cols];
}
MyMat::MyMat(int rows, int cols, double *tem){
	this->rows = rows;
	this->cols = cols;
	this->data = new double[rows*cols];
	for(int i = 0;i<rows*cols;i++)
		this->data[i] = tem[i];
}

MyMat::~MyMat(void)
{
	if(this->data)delete []data;
}

MyMat::MyMat(const MyMat &b){
	this->rows = b.rows;
	this->cols = b.cols;
	this->data = new double[rows*cols];
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			this->data[i*cols+j] = b.data[i*cols+j];
}
MyMat &MyMat::operator =(const MyMat &b){
	if(this->rows != b.rows||this->cols != b.cols){
		printf("operator = left's rows/cols is not the same as right's\n");
	}
	for(int i = 0;i<this->rows;i++)
		for(int j = 0;j<this->cols;j++)
			this->data[i*cols+j] = b.data[i*cols+j];
	return *this;
}
MyMat MyMat::operator *(MyMat &b){
	MyMat temp(rows,b.cols);
	if(this->cols!=b.rows){
		printf("operator * left's cols is not the same as right's rows\n");
		return temp;
	}
	for(int i = 0;i<this->rows;i++){
		for(int j = 0;j<b.cols;j++){
			double sum = 0;
			for(int k = 0;k<this->cols;k++)
				sum += this->data[i*this->cols+k] * b.data[k*b.cols+j];
			temp.data[i*temp.cols+j] = sum;
		}
	}
	return temp;
}
MyMat MyMat::operator *(double b){
	MyMat temp(rows,cols);
	for(int i = 0;i<rows;i++)
		for(int j  =0;j<cols;j++)
			temp.data[i*this->cols+j] = this->data[i*this->cols+j] * b;
	return temp;
}
MyMat &MyMat::operator *=(double b){
	for(int i = 0;i<rows;i++)
		for(int j  =0;j<cols;j++)
			this->data[i*this->cols+j] *= b;
	return *this;
}
MyMat MyMat::operator +(MyMat &b){
	MyMat temp(rows,cols);
	if(this->cols!=b.cols||this->rows!=b.rows){
		printf("operator + left's cols/rows is not the same as right's\n");
		return temp;
	}
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			temp.data[i*temp.cols+j] = this->data[i*this->cols+j] + b.data[i*b.cols+j];
	return temp;
}
MyMat &MyMat::operator +=(MyMat &b){
	if(this->cols!=b.cols||this->rows!=b.rows){
		printf("operator + left's cols/rows is not the same as right's\n");
		return *this;
	}
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			this->data[i*this->cols+j] += b.data[i*b.cols+j];
	return *this;
}
MyMat MyMat::operator +(double b){
	MyMat temp(*this);
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			temp.data[i*temp.cols+j] += b;
	return temp;
}
MyMat &MyMat::operator +=(double b){
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			this->data[i*this->cols+j] += b;
	return *this;
}

MyMat MyMat::operator -(MyMat &b){
	MyMat temp(rows,cols);
	if(this->cols!=b.cols||this->rows!=b.rows){
		printf("operator - left's cols/rows is not the same as right's\n");
		return temp;
	}
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			temp.data[i*temp.cols+j] = this->data[i*this->cols+j] - b.data[i*b.cols+j];
	return temp;
}
MyMat MyMat::operator -(double b){
	MyMat temp(*this);
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			temp.data[i*temp.cols+j] -= b;
	return temp;
}
MyMat &MyMat::operator -=(MyMat &b){
	if(this->cols!=b.cols||this->rows!=b.rows){
		printf("operator - left's cols/rows is not the same as right's\n");
		return *this;
	}
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			this->data[i*this->cols+j] -= b.data[i*b.cols+j];
	return *this;
}
MyMat &MyMat::operator -=(double b){
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			this->data[i*this->cols+j] -= b;
	return *this;
}
double MyMat::det(){
	if(this->cols!=this->rows){
		printf("det error:rows not equal cols\n");
		return 0;
	}
	double *temp = new double[rows*rows];
	double result = 1;
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<rows;j++)
			temp[i*rows+j] = this->data[i*rows+j];
	for(int j = 0;j<rows;j++){
		int mmaxi = j;
		for(int i = j + 1;i<rows;i++)
			if(fabs(temp[i*rows+j])>fabs(temp[mmaxi*rows+j]))
				mmaxi = i;
		if(fabs(temp[mmaxi*rows+j])<ZERO){
			result = 0;
			break;
		}
		if(mmaxi!=j){
			this->swap(temp+mmaxi*rows,temp+j*rows,rows);
			result *= -1;//交换两行，行列式的值*-1
		}
		//化成上三角矩阵
		for(int i = j+1;i<rows;i++){
			double k = temp[i*rows+j]/temp[j*rows+j];
			this->lineOper(temp+j*rows,temp+i*rows,-k,rows);
		}
	}
	for(int i = 0;i<rows;i++)
		result *= temp[i*rows+i];
	delete []temp;
	return result;
}
double &MyMat::at(int i, int j){
	return this->data[i*this->cols+j];
}
void MyMat::swap(double *a,double *b){
	double temp = *a;
	*a = *b;
	*b = temp;
}
void MyMat::swap(double *a, double *b, int n){
	double temp;
	for(int i = 0;i<n;i++){
		temp = a[i];
		a[i] = b[i];
		b[i] = temp;
	}
}
void MyMat::lineOper(double *a, double *b, double k, int n){
	for(int i = 0;i<n;i++)
		b[i] += a[i] * k;
}
MyMat MyMat::transpose(){
	MyMat temp(cols,rows);
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			temp.data[j*temp.cols+i] = this->data[i*this->cols+j];
	return temp;
}
bool MyMat::inverse(MyMat &res){
	MyMat temp(*this);
	if(this->rows!=this->cols){
		printf("inverse error:rows not equal cols\n");
		return false;
	}
	for(int i = 0;i<rows;i++){
		for(int j = 0;j<rows;j++)
			res.data[i*rows+j] = 0;
		res.data[i*rows+i] = 1;
	}
	//化成上三角矩阵
	for(int j = 0;j<rows;j++){
		int mmaxi = j;
		for(int i = j + 1;i<rows;i++)
			if(fabs(temp.data[i*rows+j])>fabs(temp.data[mmaxi*rows+j]))
				mmaxi = i;
		if(fabs(temp.data[mmaxi*rows+j])<ZERO){
			return false;
		}
		if(mmaxi!=j){
			this->swap(temp.data + j*rows,temp.data + mmaxi * rows,rows);
			this->swap(res.data + j*rows,res.data + mmaxi * rows,rows);
		}
		for(int i = j + 1;i<rows;i++){
			double k = temp.data[i*rows+j] / temp.data[j*rows+j];
			this->lineOper(temp.data + j*rows,temp.data + i*rows,-k,rows);
			this->lineOper(res.data + j*rows,res.data + i*rows,-k,rows);
		}
	}
	//化成对角阵
	for(int j = rows - 1;j>=0;j--){
		for(int i = j - 1;i>=0;i--){
			double k = temp.data[i*rows + j] / temp.data[j*rows+j];
			this->lineOper(temp.data + j*rows,temp.data + i* rows,-k,rows);
			this->lineOper(res.data + j*rows,res.data + i*rows,-k,rows);
		}
	}
	//归一化
	for(int i = 0;i<rows;i++){
		for(int j = 0;j<rows;j++){
			double k = 1/temp.data[i*rows+i];
			res.data[i*rows+j] *= k;
		}
	}
	return true;
}
void MyMat::show(){
	for(int i = 0;i<rows;i++){
		for(int j  =0;j<cols;j++){
			printf("%lf\t",this->data[i*this->cols+j]);
		}
		printf("\n");
	}
}

bool MyMat::QR(MyMat &Q,MyMat &R){
	if(this->cols!=this->rows)
		return false;//不是方阵
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<cols;j++)
			Q.at(i,j) = 0;
	for(int i = 0;i<rows;i++)
		Q.at(i,i) = 1;
	R = (*this);
	double *ai = new double[this->rows];
	double *aj = new double[this->rows];
	double *qi = new double[this->rows];
	double *qj = new double[this->rows];
	for(int j = 0;j<rows;j++){
		for(int i = j+1;i<rows;i++){
			if(fabs(R.at(i,j))<ZERO)
				continue;
			double sq = sqrt(R.at(j,j)*R.at(j,j)+R.at(i,j)*R.at(i,j));
			double c =	R.at(j,j)/sq;
			double s = R.at(i,j)/sq;

			for(int k = 0;k<rows;k++){
				aj[k] = c*R.at(j,k) + s*R.at(i,k);
				qj[k] = c*Q.at(j,k) + s*Q.at(i,k);
				ai[k] = -s*R.at(j,k) + c*R.at(i,k);
				qi[k] = -s*Q.at(j,k) + c*Q.at(i,k);
			}
			for(int k = 0;k<rows;k++){
				R.at(i,k) = ai[k];
				R.at(j,k) = aj[k];
				Q.at(i,k) = qi[k];
				Q.at(j,k) = qj[k];
			}
			
		}
	}
	delete []ai;
	delete []aj;
	delete []qi;
	delete []qj;
	Q = Q.transpose();
	return true;
}
void MyMat::SVD(MyMat &U,MyMat &S,MyMat &V){
	MyMat A = (*this);
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<rows;j++)
			V.at(i,j) = 0;
	for(int i = 0;i<rows;i++)
		V.at(i,i) = 1;
	double *tempa = new double[rows];
	double *tempv = new double[rows];
	double er = 1;
	while(fabs(er)>ZERO){
		er = 0;
		for(int i = 0;i<rows;i++){
			for(int j = i+1;j<rows;j++){
				double e = 0,d = 0,b = 0;
				for(int k = 0;k<rows;k++){
					e += A.at(k,i) * A.at(k,j);
					d += A.at(k,i) * A.at(k,i);
					b += A.at(k,j) * A.at(k,j);
				}
				if(fabs(e)>ZERO){
					double aa = 2*e;
					double bb = d - b;
					double rr = sqrt(aa*aa+bb*bb);
					/*double tan2a = (b-d)/(2*e);
					double tana = (sqrt(1+tan2a*tan2a) - 1)/tan2a;
					double c =1.0/(1+tana*tana);
					double s = tana*c;
						*/
					double c,s;
					if(bb>=0){
						c = sqrt((bb+rr)/(2*rr));
						s = aa/(2*rr*c);
					}else {
						s = sqrt((rr-bb)/(2*rr));
						c = aa/(2*rr*s);
					}
					for(int k = 0;k<rows;k++){
						tempa[k] = c*A.at(k,i) + s*A.at(k,j);
						A.at(k,j) = -s*A.at(k,i) + c*A.at(k,j);
						tempv[k] = c*V.at(k,i) + s*V.at(k,j);
						V.at(k,j) = -s*V.at(k,i) + c*V.at(k,j);
					}
					for(int k = 0;k<rows;k++){
						A.at(k,i) = tempa[k];
						V.at(k,i) = tempv[k];
					}
					if(fabs(e)>er)er = fabs(e);
				}
			}
		}
	}
	for(int i = 0;i<rows;i++)
		for(int j = 0;j<rows;j++)
			S.at(i,j) = 0;
	for(int i = 0;i<rows;i++){
		double sum = 0;
		for(int j = 0;j<rows;j++)
			sum += A.at(j,i) * A.at(j,i);
		S.at(i,i) = sqrt(sum);
	}
	for(int j = 0;j<rows;j++)
		for(int i = 0;i<rows;i++)
			U.at(i,j) = A.at(i,j) / S.at(j,j);
	delete []tempa;
	delete []tempv;
}


void MyMat::powermethod(double &y,MyMat &Y){
	Y.at(0,0) = 1;
	for(int i = 1;i<rows;i++)
		Y.at(i,0) = 0;
	MyMat ty(rows,1);
	double er = 1;
	int cou = 0;
	while(er>ZERO){
		er = 0;
		double mmax = fabs(Y.at(0,0));
		double mmaxindex = 0;
		for(int i = 1;i<rows;i++){
			if(fabs(Y.at(i,0))>mmax){
				mmax = fabs(Y.at(i,0));
				mmaxindex = i;
			}
		}
		//让模最大的维度为1
		//如果采用单位化的话，设计平方和运算，将导致溢出
		for(int i = 0;i<rows;i++){
			Y.at(i,0) /= mmax;
		}
		ty = (*this)*Y;
		y = ty.at(mmaxindex,0)/Y.at(mmaxindex,0);
		for(int i = 0;i<rows;i++){
			if(fabs(ty.at(i,0)-y*Y.at(i,0))>er)
				er = fabs(ty.at(i,0)-y*Y.at(i,0));
		}
		if(er>ZERO){
			Y = ty;
			cou++;
		}
	}
}
void MyMat::test(){
	double temp[]={
		 0,1,2,3,
		 4,5,6,7
	};
	printf("测试构造函数\n");
	MyMat mat(2,4,temp);
	mat.show();
	printf("\n");
	printf("测试拷贝构造函数\n");
	MyMat matb = mat;
	matb.show();
	printf("\n");
	printf("测试=运算符\n");
	MyMat matc(2,4);
	matc = mat;
	matc.show();
	printf("\n");
	printf("测试矩阵乘法运算符\n");
	matc.show();
	printf("*\n");
	mat.transpose().show();
	printf("=\n");
	(matc*mat.transpose()).show();
	printf("\n");
	printf("测试数乘乘法运算符\n");
	matc.show();
	printf("*-3.1\n=\n");
	(matc*-3.1).show();
	printf("\n");
	printf("测试数乘乘法运算符\n");
	(matb*=-3.1).show();
	printf("\n");
	printf("测试矩阵加法\n");
	matb.show();
	printf("+\n");
	matb.show();
	printf("=\n");
	(matb+matb).show();
	printf("\n");
	printf("测试矩阵加法\n");
	matb.show();
	printf("+100\n=\n");
	(matb+100).show();
	printf("\n");
	printf("测试矩阵加法\n");
	(matb+=100).show();
	printf("\n");
	printf("测试矩阵加法\n");
	matb.show();
	printf("+\n");
	mat.show();
	printf("=\n");
	(matb+=mat).show();
	printf("\n");
	printf("测试矩阵减法\n");
	matb.show();
	printf("-\n");
	matb.show();
	printf("=\n");
	(matb-matb).show();
	printf("\n");
	printf("测试矩阵减法\n");
	matb.show();
	printf("-100\n=\n");
	(matb-100).show();
	printf("\n");
	printf("测试矩阵减法\n");
	matb.show();
	printf("-\n");
	matb.show();
	printf("=\n");
	(matb-=matb).show();
	printf("\n");
	printf("测试矩阵减法\n");
	(matb-=100).show();
	printf("\n");
	printf("测试矩阵求逆\n");
	MyMat matt = mat * mat.transpose();
	matt.show();
	printf("的逆\n");
	MyMat mat_(matt.rows,matt.rows);
	matt.inverse(mat_);
	mat_.show();
	printf("\n");
	printf("测试矩阵行列式\n");
	matt.show();
	printf("的行列式\n");
	printf("%lf",matt.det());
	printf("\n");
	printf("测试矩阵转置\n");
	mat.transpose().show();
	printf("\n");
	printf("QR分解测试\n");
	double tem[]={
		5,-3,2,
		6,-4,4,
		4,-4,5
	};
	MyMat matd(3,3,tem);
	MyMat Q(3,3);
	MyMat R(3,3);
	matd.QR(Q,R);
	printf("A:\n");
	matd.show();
	printf("Q:\n");
	Q.show();
	printf("R:\n");
	R.show();
	printf("Q*R\n");
	(Q*R).show();
	printf("\n");
	printf("测试SVD\n");
	printf("A:\n");
	(matd.transpose()*matd).show();
	//matd.show();
	MyMat U(3,3);
	MyMat S(3,3);
	MyMat V(3,3);
	(matd.transpose()*matd).SVD(U,S,V);
	//matd.SVD(U,S,V);
	printf("U:\n");
	U.show();
	printf("S:\n");
	S.show();
	printf("V:\n");
	V.show();
	printf("U*S*V'\n");
	(U*S*V.transpose()).show();
	printf("\n");
	printf("测试SVD\n");
	/*double tem2[]={
		28.154066,	-4.206267,	-19.178119,	-0.013366,	0.031923,	-0.000014,
		-4.206267,	49.874168,	-11.172110,	-0.037940,	0.001894,	0.000004,
		-19.178119,	-11.172110,	20.578652,	0.024738,	-0.022769,	0.000005,
		-0.013366,	-0.037940,	0.024738,	0.000048,	-0.000024,	0.000000,
		0.031923,	0.001894,	-0.022769,	-0.000024,	0.000042,	-0.000000,
		-0.000014,	0.000004,	0.000005,	0.000000,	-0.000000,	0.000000
	};*/
	double tem2[]={
		189.747921,	36.599067,	-158.394399,	49022.801474,	-18422.115264,	2644019.156112	,
		36.599067,	173.547764,	-2.217607,	19752.175165,	20566.706254	,1823610.870816	,
		-158.394399,	-2.217607,	138.821788,	-39496.951087,	19172.458726,	-2056017.083326	,
		49022.801474,	19752.175165,	-39496.951087,	14681827.889290,	-2978145.332130,	977237826.225876	,
		-18422.115264,	20566.706254,	19172.458726,	-2978145.332130,	5584132.645830,	-6074819.907659	,
		2644019.156112,	1823610.870816,	-2056017.083326,	977237826.225876,	-6074819.907659,	81309865204.910492	
	};
	MyMat matf(6,6,tem2);
	matf = matf*0.000000001;
	MyMat u(6,6);
	MyMat s(6,6);
	MyMat v(6,6);
	printf("A:\n");
	matf.show();
	matf.SVD(u,s,v);
	printf("u:\n");
	u.show();
	printf("s:\n");
	s.show();
	printf("v:\n");
	v.show();
	printf("\n");
	void testpower();
	testpower();
}

void testpower(){
	double tem[]={
		5,-3,2,
		6,-4,4,
		4,-4,5
	};
	double tem2[]={
		189.747921,	36.599067,	-158.394399,	49022.801474,	-18422.115264,	2644019.156112	,
		36.599067,	173.547764,	-2.217607,	19752.175165,	20566.706254	,1823610.870816	,
		-158.394399,	-2.217607,	138.821788,	-39496.951087,	19172.458726,	-2056017.083326	,
		49022.801474,	19752.175165,	-39496.951087,	14681827.889290,	-2978145.332130,	977237826.225876	,
		-18422.115264,	20566.706254,	19172.458726,	-2978145.332130,	5584132.645830,	-6074819.907659	,
		2644019.156112,	1823610.870816,	-2056017.083326,	977237826.225876,	-6074819.907659,	81309865204.910492	
	};
	MyMat matd(6,6,tem2);
	MyMat matd_(6,6);
	matd.inverse(matd_);
	double y;
	MyMat Y(6,1);
	printf("原矩阵\n");
	matd.show();
	matd_.powermethod(y,Y);
	printf("最小特征值:%.8lf\n",y);
	printf("对应的特征向量(未归一化)\n");
	Y.show();

}
