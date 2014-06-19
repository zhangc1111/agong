#include "matcher.h"

#define PI 3.1415926

Matcher::Matcher()
{

	//template_point_count.resize(99);
	//template_location.resize(99);
	//template_direction.resize(99);
	//template_direction_length.resize(99);
	//angle_size.resize(99);
	//step.resize(99);
	//m_size.resize(99);
}

Matcher::~Matcher()
{

}

int Matcher::CreateModel(byte *image, int width, int height, int MaxLevel, int sobel_size, float step_length, double threshold1, double threshold2)
{
	Mat template_image = Mat(height, width, CV_8U, image);

	if(template_image.empty())
		return 1;

	//(*point_center[ModelIndex]).x = (width - 1)/2.0;
	//(*point_center[ModelIndex]).y = (height - 1)/2.0;

	//if (max != oldmax)
	//{
	//	delete orb;
	//	orb = new ORB(max, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31);
	//	oldmax = max;
	//}

	//(*orb)(input_image, Mat(), keyPoints_1[ModelIndex], *descriptors_1[ModelIndex]);
	//
	//if(keyPoints_1[ModelIndex].empty())
	//	return 2;



	template_point_count = new int[MaxLevel];
	template_location = new LOCATION**[MaxLevel];
	template_direction = new DIRECTION**[MaxLevel];
	template_direction_length = new float*[MaxLevel];
	m_size = sobel_size;

	Mat_<uchar> *input_template = new Mat_<uchar>[MaxLevel];

	Mat *template_sobel_x = new Mat[MaxLevel];
	Mat *template_sobel_y = new Mat[MaxLevel];
	LOCATION *template_center = new LOCATION[MaxLevel];
	//canny images
	Mat *canny_edges = new Mat[MaxLevel];

	//angle_size[0] = 629;
	//angle_size[1] = 315;
	//angle_size[2] = 158;
	//angle_size[3] = 79;
	//angle_size[4] = 40;

	angle_size = new int[MaxLevel];
	step = new float[MaxLevel];
	step[0] = step_length;
	angle_size[0] = (int)(360/step[0]+0.5);

	for(int i = 1;i<MaxLevel;i++)
	{
		step[i] = step[i-1]*2; 
		angle_size[i] = (int)(360/step[i]+0.5);
	}

	//模板为空，返回

	if (template_image.empty())
	{
		return 0;
	}

	//转化为单通道图像
	if(template_image.channels() != 1)
		cvtColor(template_image, input_template[0], CV_BGR2GRAY);
	else
		input_template[0] = template_image.clone();

	//生成图像金字塔
	for(int i = 0; i< MaxLevel-1;i++)
	{
		pyrDown(input_template[i], input_template[i+1]);
	}
	//提取各层sobel图像
	for(int i = 0; i< MaxLevel;i++)
	{
		Canny(input_template[i], canny_edges[i], (50 - i*10), (60-i*10));
		Sobel(input_template[i], template_sobel_x[i], CV_16S, 1, 0, m_size);
		Sobel(input_template[i], template_sobel_y[i], CV_16S, 0, 1, m_size);
	}


	//统计每一层的sobel点个数
	int x, y, direction_length;

	for(int i = 0; i<MaxLevel; i++)
	{
		template_point_count[i] = 0;
		for( size_t nrow = 0; nrow < input_template[i].rows; nrow++)  
		{  
			for(size_t ncol = 0; ncol < input_template[i].cols; ncol++)  
			{  
				x = template_sobel_x[i].at<short>(nrow, ncol);
				y = template_sobel_y[i].at<short>(nrow, ncol);
				direction_length = (int)x*(int)x + (int)y*(int)y;
				
				if(canny_edges[i].at<uchar>(nrow, ncol) > 0 && direction_length > 0)
					template_point_count[i]++;
				/*if(direction_length > 5000)
					template_point_count[i]++;*/
			}  	
		}
	}


	//计算每层的sobel点位置，对应的方向向量，及方向向量的长度

	for (int i = 0; i < MaxLevel; i++)
	{
		template_location[i] = new LOCATION*[angle_size[i]];
		template_direction[i] = new DIRECTION*[angle_size[i]];


		template_location[i][0] = new LOCATION[template_point_count[i]];
		template_direction[i][0] = new DIRECTION[template_point_count[i]];
		template_direction_length[i] = new float[template_point_count[i]];

		template_center[i].x = input_template[i].size().width/2;
		template_center[i].y = input_template[i].size().height/2;

		template_point_count[i] = 0;
		for( size_t nrow = 0; nrow < input_template[i].rows; nrow++)  
		{  
			for(size_t ncol = 0; ncol < input_template[i].cols; ncol++)  
			{  
				x = template_sobel_x[i].at<short>(nrow, ncol);
				y = template_sobel_y[i].at<short>(nrow, ncol);
				direction_length = (int)x*(int)x + (int)y*(int)y;
				/*if(direction_length > 5000)*/
				if(canny_edges[i].at<uchar>(nrow, ncol) != 0 && direction_length > 0)
				{
					template_location[i][0][template_point_count[i]].x = ncol - template_center[i].x;
					template_location[i][0][template_point_count[i]].y = nrow - template_center[i].y;

					template_direction[i][0][template_point_count[i]].x = template_sobel_x[i].at<short>(nrow, ncol);
					template_direction[i][0][template_point_count[i]].y = template_sobel_y[i].at<short>(nrow, ncol);

					template_direction_length[i][template_point_count[i]] = 1.0/sqrt((float)direction_length);
					template_point_count[i]++;
				}
			}  	
		}

	}


	//计算每层旋转后的sobel点位置，对应的方向向量
	//float delta_angle = 0.573;
	//delta_angle = 0.573*2;
	for (int index = 0; index < MaxLevel; index++)
	{

		for (int i = 1; i < angle_size[index]; i++)
		{
			float cosfactor = cos(i*step[index]*PI/180);
			float sinfactor = sin(i*step[index]*PI/180);

			template_location[index][i] = new LOCATION[template_point_count[index]];
			template_direction[index][i] = new DIRECTION[template_point_count[index]];
			for (int j = 0; j < template_point_count[index];j++)
			{
				template_location[index][i][j].x = template_location[index][0][j].x * cosfactor - template_location[index][0][j].y * sinfactor;
				template_location[index][i][j].y = template_location[index][0][j].x * sinfactor + template_location[index][0][j].y * cosfactor;

				template_direction[index][i][j].x = template_direction[index][0][j].x * cosfactor - template_direction[index][0][j].y * sinfactor;
				template_direction[index][i][j].y = template_direction[index][0][j].x * sinfactor + template_direction[index][0][j].y * cosfactor;

			}
		}

		//delta_angle = delta_angle * 2;
	}
	return 0;
}


int Matcher::MatchMulti(byte *input_image, int width, int height, int MaxLevel, float min_angle, float max_angle, int max_instance, float score_threshold, float* x, float* y, float* angle, float* score)
{


	Mat des_image = Mat(height, width, CV_8U, input_image);


	if (des_image.empty())
		return 1;

	float user_threshold = 0.8;
	float g = 0.9;
	float f;
	f = (1-g*user_threshold)/(1-user_threshold);
	int match_count = 0;

	//Mat_<uchar> image[5];
	//int image_width[5];
	//int image_height[5];

	//Mat image_sobel_x[5]; 
	//Mat image_sobel_y[5];

	Mat_<uchar> *image = new Mat_<uchar>[MaxLevel];
	int *image_width = new int[MaxLevel];
	int *image_height = new int[MaxLevel];

	Mat *image_sobel_x = new Mat[MaxLevel]; 
	Mat *image_sobel_y = new Mat[MaxLevel];


	int nRows, nCols;
	short* px;
	short* py;

	int length;
	//int myx[5];
	//int myy[5];
	//int myangle_index[5];
	//float m_max[5];

	//int *myx = new int[MaxLevel];
	//int *myy = new int[MaxLevel];
	//int *myangle_index = new int[MaxLevel];
	//float *m_max = new float[MaxLevel];

	int** myx = new int*[MaxLevel];
	int** myy = new int*[MaxLevel];
	int** myangle_index = new int*[MaxLevel];
	float** m_max = new float*[MaxLevel];


	int temp_x_index, temp_y_index;

	Point point1, point2, point3, point4, point1_new, point2_new, point3_new, point4_new;
	float final_angle;

	//存放图像中的sobel向量与长度
	//DIRECTION ***image_direction_map = new DIRECTION **[MaxLevel];
	//float ***image_direction_length = new float **[MaxLevel];

	DIRECTION** image_direction_map = new DIRECTION*[MaxLevel];
	float** image_direction_length = new float*[MaxLevel];
	
	//显示结果图像
	Mat output = des_image.clone();
	
	//转化为单通道图像
	if(des_image.channels() != 1)
		cvtColor(des_image, image[0], CV_BGR2GRAY);
	else
		image[0] = des_image.clone();

	//生成图像金字塔
	for(int i=0;i<MaxLevel-1;i++)
	{
		pyrDown(image[i], image[i+1]);
	}
	

	for(int level_index = 0;level_index < MaxLevel;level_index++)
	{
			
		myx[level_index] = new int[10];
		myy[level_index] = new int[10];
		myangle_index[level_index] = new int[10];
		m_max[level_index] = new float[10];
			
		//提取各层sobel图像
		Sobel(image[level_index], image_sobel_x[level_index], CV_16S, 1, 0, m_size);
		Sobel(image[level_index], image_sobel_y[level_index], CV_16S, 0, 1, m_size);

		//Canny(image[level_index], canny_image[level_index], 50, 128);
		
		image_width[level_index] = image[level_index].size().width;
		image_height[level_index] = image[level_index].size().height;

		//为sobel向量map和向量长度map分配指针内存
		//image_direction_map[level_index] = new DIRECTION *[image_height[level_index]];
		//image_direction_length[level_index] = new float *[image_height[level_index]];
		
		image_direction_map[level_index] = new DIRECTION[image_height[level_index]*image_width[level_index]];
		image_direction_length[level_index] = new float[image_height[level_index]*image_width[level_index]];
		
		//将sobel向量和长度存入map中
		nRows = image_height[level_index];
		nCols = image_width[level_index];

		for( int i = 0; i < nRows; ++i)  
		{  
			px = image_sobel_x[level_index].ptr<short>(i);
			py = image_sobel_y[level_index].ptr<short>(i);

			//image_direction_map[level_index][i] = new DIRECTION[nCols];
			//image_direction_length[level_index][i] = new float[nCols];

			for ( int j = 0; j < nCols; ++j)  
			{  
				//image_direction_map[level_index][i][j].x = px[j];
				//image_direction_map[level_index][i][j].y = py[j];

				image_direction_map[level_index][i*nCols+j].x = px[j];
				image_direction_map[level_index][i*nCols+j].y = py[j];

				length = (int)px[j]*(int)px[j] + (int)py[j]*(int)py[j];
				
				//canny selection
				if(length > 100)
				//if(pcanny[j] > 0)
				{
					//image_direction_length[level_index][i][j] = 1.0/sqrt((float)length);
					image_direction_length[level_index][i*nCols+j] = 1.0/sqrt((float)length);
					//point_image_map[level_index].at<uchar>(i, j) = 255;
				}
				else
					//image_direction_length[level_index][i][j] = 0;
					image_direction_length[level_index][i*nCols+j] = 0;
			}  
		}
	
	}



	//从最粗糙层次开始匹配
	vector<double> point_score;
	vector<Point> point_with_zero_score;
	vector<float> temp_length_list;
	vector<float> image_length_list;
	
	nRows = image_height[MaxLevel-1];
	nCols = image_width[MaxLevel-1];
	myx[MaxLevel-1][0] = 0;
	myy[MaxLevel-1][0] = 0;
	myangle_index[MaxLevel-1][0] = 0;
	m_max[MaxLevel-1][0] = 0;
	float nt_n = template_point_count[MaxLevel-1] * user_threshold - template_point_count[MaxLevel-1];

	float* mj_list = new float[nRows * nCols];
	int* mj_angle_list = new int[nRows * nCols];

	for(int yy = 0; yy < nRows; ++yy)
	//for(int y = -y_min[MaxLevel-1]; y < nRows - y_max[MaxLevel -1]; ++y)
	{
		for (int xx = 0; xx< nCols; ++xx)
		//for (int x = -x_min[MaxLevel-1]; x< nCols - x_max[MaxLevel -1]; ++x)
		{
			float temp_mj_max = 0;
			int temp_max_x = 0;
			int temp_max_y = 0;
			int temp_max_angle = 0;
				
			for (int angle_index = 0; angle_index < angle_size[MaxLevel-1]; angle_index++)
			{
					
				float actual_angle = angle_index * step[MaxLevel-1];
				if(actual_angle > 180)
					actual_angle = actual_angle - 360;

				if(actual_angle < min_angle || actual_angle > max_angle)
					continue;

				float mj = 0;
				DIRECTION temp_template_direction, temp_image_direction;
				LOCATION temp_template_location;
				

				for (int template_point_index = 0; template_point_index < template_point_count[MaxLevel-1]; template_point_index++)
				{
					temp_template_direction = template_direction[MaxLevel-1][angle_index][template_point_index];
					temp_template_location = template_location[MaxLevel-1][angle_index][template_point_index];
					
					temp_x_index = temp_template_location.x + xx;
					temp_y_index = temp_template_location.y + yy;

					if(!(temp_x_index >=0 && temp_x_index < nCols && temp_y_index>=0 && temp_y_index < nRows))
						continue;

						
					//temp_image_direction = image_direction_map[MaxLevel-1][temp_y_index][temp_x_index];
					temp_image_direction = image_direction_map[MaxLevel-1][temp_y_index*nCols+temp_x_index];

					//mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[MaxLevel-1][template_point_index] * image_direction_length[MaxLevel-1][temp_y_index][temp_x_index]);
					mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[MaxLevel-1][template_point_index] * image_direction_length[MaxLevel-1][temp_y_index*nCols+temp_x_index]);
					if(mj < min(nt_n + f*(template_point_index+1), user_threshold*((template_point_index+1))))
						break;
					
				}
				mj = mj/template_point_count[MaxLevel-1];
				//qDebug()<<"hh "<<mj;
				if(mj>0.50)
				{
					//qDebug()<<"hh "<<mj;
					if (mj> temp_mj_max)
					{
						temp_mj_max = mj;
						temp_max_x = xx;
						temp_max_y = yy;
						temp_max_angle = angle_index;
					

					}
				}




			}

			mj_list[yy*nCols+xx] = temp_mj_max;
			mj_angle_list[yy*nCols+xx] = temp_max_angle;
			if(temp_mj_max > m_max[MaxLevel-1][0])
			{
				myx[MaxLevel-1][0] = temp_max_x;
				myy[MaxLevel-1][0] = temp_max_y;
				myangle_index[MaxLevel-1][0] = temp_max_angle;
				m_max[MaxLevel-1][0] = temp_mj_max;
			}
		}
	}

	int max_point_count = 0;
	for(max_point_count = 1; max_point_count < max_instance * 2; max_point_count++)
	{
		//clear m_max and its neighbour to search second largest.
		for(int i = -5; i<=5;i++)
			for(int j = -5;j<=5;j++)
			{
				int tempindex = ((myy[MaxLevel-1][max_point_count-1])+i)*nCols + myx[MaxLevel-1][max_point_count-1]+j;
				if (tempindex >=0 && tempindex < nRows *nCols)
					mj_list[tempindex] = 0;
			}


			//for test...  second score......................................
			myx[MaxLevel-1][max_point_count] = 0;
			myy[MaxLevel-1][max_point_count] = 0;
			myangle_index[MaxLevel-1][max_point_count] = 0;
			m_max[MaxLevel-1][max_point_count] = 0;

			for(int yy = 0; yy < nRows; ++yy)
				//for(int y = -y_min[MaxLevel-1]; y < nRows - y_max[MaxLevel -1]; ++y)
			{
				for (int xx = 0; xx< nCols; ++xx)
					//for (int x = -x_min[MaxLevel-1]; x< nCols - x_max[MaxLevel -1]; ++x)
				{
					if( mj_list[yy*nCols + xx]> m_max[MaxLevel-1][max_point_count])
					{
						m_max[MaxLevel-1][max_point_count] = mj_list[yy*nCols + xx];
						myx[MaxLevel-1][max_point_count] = xx;
						myy[MaxLevel-1][max_point_count] = yy;
						myangle_index[MaxLevel-1][max_point_count] = mj_angle_list[yy*nCols + xx];
					}

				}
			}

			if(m_max[MaxLevel-1][max_point_count] < 0.7)
				break;
	}

	delete mj_list;
	delete mj_angle_list;

	mj_list = NULL;
	mj_angle_list = NULL;


	//level  匹配
	int temp_angle_index;

	vector<Point> point_with_zero_score_in_level0;

	for(int count = 0; count < max_point_count; count++)
	{

		for (int level_index = MaxLevel-2; level_index >= 0;level_index--)
		{
			nRows = image_height[level_index];
			nCols = image_width[level_index];
			myx[level_index][count] = 0;
			myy[level_index][count] = 0;
			if(myx[level_index+1][count] == 0 || myy[level_index+1][count] == 0)
				break;
			myangle_index[level_index][count] = 0;
			m_max[level_index][count] = 0;

			nt_n = template_point_count[level_index] * user_threshold - template_point_count[level_index];
			for(int yy = myy[level_index+1][count]*2-3; yy <= myy[level_index+1][count]*2+3; ++yy)
			{
				for (int xx = myx[level_index+1][count]*2-3; xx<=myx[level_index+1][count]*2+3; ++xx)
				{

					for (int angle_index = myangle_index[level_index+1][count]*2 - 2; angle_index <= myangle_index[level_index+1][count]*2 + 2; angle_index++)
					{
						vector<double> temp_score;
						temp_score.clear();
						temp_angle_index = angle_index;
						if (angle_index < 0)
							temp_angle_index = angle_index + angle_size[level_index];

						if (angle_index >= angle_size[level_index])
							temp_angle_index = angle_index - angle_size[level_index];
						//vector<Point> temp_point_with_zero_score;
						//temp_point_with_zero_score.clear();


						float mj = 0;
						DIRECTION temp_template_direction, temp_image_direction;
						LOCATION temp_template_location;
						for (int template_point_index = 0; template_point_index < template_point_count[level_index]; template_point_index++)
						{
							temp_template_direction = template_direction[level_index][temp_angle_index][template_point_index];
							temp_template_location = template_location[level_index][temp_angle_index][template_point_index];

							temp_x_index = temp_template_location.x + xx;
							temp_y_index = temp_template_location.y + yy;

							if(!(temp_x_index >=0 && temp_x_index < image_width[level_index] && temp_y_index>=0 && temp_y_index < image_height[level_index]))
								continue;


							//temp_image_direction = image_direction_map[level_index][temp_y_index][temp_x_index];
							temp_image_direction = image_direction_map[level_index][temp_y_index*nCols+temp_x_index];

							//if (level_index == 0)
							//{
							//	double ans = abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[level_index][template_point_index] * image_direction_length[level_index][temp_y_index][temp_x_index]);
							//	if(ans < 0.00000001)
							//	{
							//		temp_point_with_zero_score.push_back(Point(temp_x_index, temp_y_index));
							//		//temp_point_with_zero_score.push_back(Point(temp_template_direction.x, temp_template_direction.y));
							//	}
							//		
							//
							//}

							//mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[level_index][template_point_index] * image_direction_length[level_index][temp_y_index][temp_x_index]);
							mj = mj + fabs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[level_index][template_point_index] * image_direction_length[level_index][temp_y_index*nCols+temp_x_index]);
						}
						mj = mj/template_point_count[level_index];

						//qDebug()<<x<<y<<temp_angle_index<<mj;
						if (mj> m_max[level_index][count])
						{
							m_max[level_index][count] = mj;
							myx[level_index][count] = xx;
							myy[level_index][count] = yy;
							myangle_index[level_index][count] = temp_angle_index;

							//point_with_zero_score_in_level0.clear();
							//point_with_zero_score_in_level0.insert(point_with_zero_score_in_level0.end(), temp_point_with_zero_score.begin(),temp_point_with_zero_score.end());

						}
					}
				}
			}

		}
	}
		
	
	for (int count = 0; count < max_point_count; count++)
	{
		final_angle = myangle_index[0][count]*step[0];
		if(final_angle > 180)
			final_angle = final_angle - 360;
		if(m_max[0][count] > score_threshold && final_angle > min_angle && final_angle < max_angle)
		{
			x[match_count] = myx[0][count];
			y[match_count] = myy[0][count];
			angle[match_count] = final_angle;
			score[match_count] = m_max[0][count];
			match_count++;
		}

	}
	//delete各种对象



	for(int level_index = 0;level_index < MaxLevel;level_index++)
	{

		delete[] image_direction_map[level_index];
		delete[] image_direction_length[level_index];
		delete[] myx[level_index];
		delete[] myy[level_index];
		delete[] myangle_index[level_index];
		delete[] m_max[level_index];

	}

	delete[] image_direction_map;
	delete[] image_direction_length;

	delete[] myx;
	delete[] myy;
	delete[] myangle_index;
	delete[] m_max;

	delete[] image;
	delete[] image_width;
	delete[] image_height;
	delete[] image_sobel_x;
	delete[] image_sobel_y;


	return match_count;

}


int Matcher::Match(byte *input_image, int width, int height, int MaxLevel, float min_angle, float max_angle, float score_threshold, float& x, float& y, float& angle, float& score)
{


	Mat des_image = Mat(height, width, CV_8U, input_image);


	if (des_image.empty())
		return 1;

	float user_threshold = 0.8;
	float g = 0.9;
	float f;
	f = (1-g*user_threshold)/(1-user_threshold);
	int match_count = 0;

	//Mat_<uchar> image[5];
	//int image_width[5];
	//int image_height[5];

	//Mat image_sobel_x[5]; 
	//Mat image_sobel_y[5];

	Mat_<uchar> *image = new Mat_<uchar>[MaxLevel];
	int *image_width = new int[MaxLevel];
	int *image_height = new int[MaxLevel];

	Mat *image_sobel_x = new Mat[MaxLevel]; 
	Mat *image_sobel_y = new Mat[MaxLevel];


	int nRows, nCols;
	short* px;
	short* py;

	int length;
	//int myx[5];
	//int myy[5];
	//int myangle_index[5];
	//float m_max[5];

	//int *myx = new int[MaxLevel];
	//int *myy = new int[MaxLevel];
	//int *myangle_index = new int[MaxLevel];
	//float *m_max = new float[MaxLevel];

	int** myx = new int*[MaxLevel];
	int** myy = new int*[MaxLevel];
	int** myangle_index = new int*[MaxLevel];
	float** m_max = new float*[MaxLevel];


	int temp_x_index, temp_y_index;

	Point point1, point2, point3, point4, point1_new, point2_new, point3_new, point4_new;
	float final_angle;

	//存放图像中的sobel向量与长度
	//DIRECTION ***image_direction_map = new DIRECTION **[MaxLevel];
	//float ***image_direction_length = new float **[MaxLevel];

	DIRECTION** image_direction_map = new DIRECTION*[MaxLevel];
	float** image_direction_length = new float*[MaxLevel];

	//显示结果图像
	Mat output = des_image.clone();

	//转化为单通道图像
	if(des_image.channels() != 1)
		cvtColor(des_image, image[0], CV_BGR2GRAY);
	else
		image[0] = des_image.clone();

	//生成图像金字塔
	for(int i=0;i<MaxLevel-1;i++)
	{
		pyrDown(image[i], image[i+1]);
	}


	for(int level_index = 0;level_index < MaxLevel;level_index++)
	{

		myx[level_index] = new int[10];
		myy[level_index] = new int[10];
		myangle_index[level_index] = new int[10];
		m_max[level_index] = new float[10];

		//提取各层sobel图像
		Sobel(image[level_index], image_sobel_x[level_index], CV_16S, 1, 0, m_size);
		Sobel(image[level_index], image_sobel_y[level_index], CV_16S, 0, 1, m_size);

		//Canny(image[level_index], canny_image[level_index], 50, 128);

		image_width[level_index] = image[level_index].size().width;
		image_height[level_index] = image[level_index].size().height;

		//为sobel向量map和向量长度map分配指针内存
		//image_direction_map[level_index] = new DIRECTION *[image_height[level_index]];
		//image_direction_length[level_index] = new float *[image_height[level_index]];

		image_direction_map[level_index] = new DIRECTION[image_height[level_index]*image_width[level_index]];
		image_direction_length[level_index] = new float[image_height[level_index]*image_width[level_index]];

		//将sobel向量和长度存入map中
		nRows = image_height[level_index];
		nCols = image_width[level_index];

		for( int i = 0; i < nRows; ++i)  
		{  
			px = image_sobel_x[level_index].ptr<short>(i);
			py = image_sobel_y[level_index].ptr<short>(i);

			//image_direction_map[level_index][i] = new DIRECTION[nCols];
			//image_direction_length[level_index][i] = new float[nCols];

			for ( int j = 0; j < nCols; ++j)  
			{  
				//image_direction_map[level_index][i][j].x = px[j];
				//image_direction_map[level_index][i][j].y = py[j];

				image_direction_map[level_index][i*nCols+j].x = px[j];
				image_direction_map[level_index][i*nCols+j].y = py[j];

				length = (int)px[j]*(int)px[j] + (int)py[j]*(int)py[j];

				//canny selection
				if(length > 100)
					//if(pcanny[j] > 0)
				{
					//image_direction_length[level_index][i][j] = 1.0/sqrt((float)length);
					image_direction_length[level_index][i*nCols+j] = 1.0/sqrt((float)length);
					//point_image_map[level_index].at<uchar>(i, j) = 255;
				}
				else
					//image_direction_length[level_index][i][j] = 0;
					image_direction_length[level_index][i*nCols+j] = 0;
			}  
		}

	}



	//从最粗糙层次开始匹配
	vector<double> point_score;
	vector<Point> point_with_zero_score;
	vector<float> temp_length_list;
	vector<float> image_length_list;

	nRows = image_height[MaxLevel-1];
	nCols = image_width[MaxLevel-1];
	myx[MaxLevel-1][0] = 0;
	myy[MaxLevel-1][0] = 0;
	myangle_index[MaxLevel-1][0] = 0;
	m_max[MaxLevel-1][0] = 0;
	float nt_n = template_point_count[MaxLevel-1] * user_threshold - template_point_count[MaxLevel-1];

	float* mj_list = new float[nRows * nCols];
	int* mj_angle_list = new int[nRows * nCols];

	for(int yy = 0; yy < nRows; ++yy)
		//for(int y = -y_min[MaxLevel-1]; y < nRows - y_max[MaxLevel -1]; ++y)
	{
		for (int xx = 0; xx< nCols; ++xx)
			//for (int x = -x_min[MaxLevel-1]; x< nCols - x_max[MaxLevel -1]; ++x)
		{
			float temp_mj_max = 0;
			int temp_max_x = 0;
			int temp_max_y = 0;
			int temp_max_angle = 0;

			for (int angle_index = 0; angle_index < angle_size[MaxLevel-1]; angle_index++)
			{

				float actual_angle = angle_index * step[MaxLevel-1];
				if(actual_angle > 180)
					actual_angle = actual_angle - 360;

				if(actual_angle < min_angle || actual_angle > max_angle)
					continue;

				float mj = 0;
				DIRECTION temp_template_direction, temp_image_direction;
				LOCATION temp_template_location;


				for (int template_point_index = 0; template_point_index < template_point_count[MaxLevel-1]; template_point_index++)
				{
					temp_template_direction = template_direction[MaxLevel-1][angle_index][template_point_index];
					temp_template_location = template_location[MaxLevel-1][angle_index][template_point_index];

					temp_x_index = temp_template_location.x + xx;
					temp_y_index = temp_template_location.y + yy;

					if(!(temp_x_index >=0 && temp_x_index < nCols && temp_y_index>=0 && temp_y_index < nRows))
						continue;


					//temp_image_direction = image_direction_map[MaxLevel-1][temp_y_index][temp_x_index];
					temp_image_direction = image_direction_map[MaxLevel-1][temp_y_index*nCols+temp_x_index];

					//mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[MaxLevel-1][template_point_index] * image_direction_length[MaxLevel-1][temp_y_index][temp_x_index]);
					mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[MaxLevel-1][template_point_index] * image_direction_length[MaxLevel-1][temp_y_index*nCols+temp_x_index]);
					if(mj < min(nt_n + f*(template_point_index+1), user_threshold*((template_point_index+1))))
						break;

				}
				mj = mj/template_point_count[MaxLevel-1];
				//qDebug()<<"hh "<<mj;
				if(mj>0.50)
				{
					//qDebug()<<"hh "<<mj;
					if (mj> temp_mj_max)
					{
						temp_mj_max = mj;
						temp_max_x = xx;
						temp_max_y = yy;
						temp_max_angle = angle_index;


					}
				}




			}

			mj_list[yy*nCols+xx] = temp_mj_max;
			mj_angle_list[yy*nCols+xx] = temp_max_angle;
			if(temp_mj_max > m_max[MaxLevel-1][0])
			{
				myx[MaxLevel-1][0] = temp_max_x;
				myy[MaxLevel-1][0] = temp_max_y;
				myangle_index[MaxLevel-1][0] = temp_max_angle;
				m_max[MaxLevel-1][0] = temp_mj_max;
			}
		}
	}

	int max_point_count = 0;
	for(max_point_count = 1; max_point_count < 2; max_point_count++)
	{
		//clear m_max and its neighbour to search second largest.
		for(int i = -5; i<=5;i++)
			for(int j = -5;j<=5;j++)
			{
				int tempindex = ((myy[MaxLevel-1][max_point_count-1])+i)*nCols + myx[MaxLevel-1][max_point_count-1]+j;
				if (tempindex >=0 && tempindex < nRows *nCols)
					mj_list[tempindex] = 0;
			}


			//for test...  second score......................................
			myx[MaxLevel-1][max_point_count] = 0;
			myy[MaxLevel-1][max_point_count] = 0;
			myangle_index[MaxLevel-1][max_point_count] = 0;
			m_max[MaxLevel-1][max_point_count] = 0;

			for(int yy = 0; yy < nRows; ++yy)
				//for(int y = -y_min[MaxLevel-1]; y < nRows - y_max[MaxLevel -1]; ++y)
			{
				for (int xx = 0; xx< nCols; ++xx)
					//for (int x = -x_min[MaxLevel-1]; x< nCols - x_max[MaxLevel -1]; ++x)
				{
					if( mj_list[yy*nCols + xx]> m_max[MaxLevel-1][max_point_count])
					{
						m_max[MaxLevel-1][max_point_count] = mj_list[yy*nCols + xx];
						myx[MaxLevel-1][max_point_count] = xx;
						myy[MaxLevel-1][max_point_count] = yy;
						myangle_index[MaxLevel-1][max_point_count] = mj_angle_list[yy*nCols + xx];
					}

				}
			}

			if(m_max[MaxLevel-1][max_point_count] < 0.7)
				break;
	}

	delete mj_list;
	delete mj_angle_list;

	mj_list = NULL;
	mj_angle_list = NULL;


	int temp_angle_index;

	vector<Point> point_with_zero_score_in_level0;

	for (int level_index = MaxLevel-2; level_index >= 0;level_index--)
	{
		nRows = image_height[level_index];
		nCols = image_width[level_index];
		myx[level_index][0] = 0;
		myy[level_index][0] = 0;
		if(myx[level_index+1][0] == 0 || myy[level_index+1][0] == 0)
			break;
		myangle_index[level_index][0] = 0;
		m_max[level_index][0] = 0;

		nt_n = template_point_count[level_index] * user_threshold - template_point_count[level_index];
		for(int yy = myy[level_index+1][0]*2-3; yy <= myy[level_index+1][0]*2+3; ++yy)
		{
			for (int xx = myx[level_index+1][0]*2-3; xx<=myx[level_index+1][0]*2+3; ++xx)
			{

				for (int angle_index = myangle_index[level_index+1][0]*2 - 2; angle_index <= myangle_index[level_index+1][0]*2 + 2; angle_index++)
				{
					vector<double> temp_score;
					temp_score.clear();
					temp_angle_index = angle_index;
					if (angle_index < 0)
						temp_angle_index = angle_index + angle_size[level_index];

					if (angle_index >= angle_size[level_index])
						temp_angle_index = angle_index - angle_size[level_index];
					//vector<Point> temp_point_with_zero_score;
					//temp_point_with_zero_score.clear();


					float mj = 0;
					DIRECTION temp_template_direction, temp_image_direction;
					LOCATION temp_template_location;
					for (int template_point_index = 0; template_point_index < template_point_count[level_index]; template_point_index++)
					{
						temp_template_direction = template_direction[level_index][temp_angle_index][template_point_index];
						temp_template_location = template_location[level_index][temp_angle_index][template_point_index];

						temp_x_index = temp_template_location.x + xx;
						temp_y_index = temp_template_location.y + yy;

						if(!(temp_x_index >=0 && temp_x_index < image_width[level_index] && temp_y_index>=0 && temp_y_index < image_height[level_index]))
							continue;


						//temp_image_direction = image_direction_map[level_index][temp_y_index][temp_x_index];
						temp_image_direction = image_direction_map[level_index][temp_y_index*nCols+temp_x_index];


						//mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[ModelIndex][level_index][template_point_index] * image_direction_length[level_index][temp_y_index][temp_x_index]);
						mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[level_index][template_point_index] * image_direction_length[level_index][temp_y_index*nCols+temp_x_index]);
					}
					mj = mj/template_point_count[level_index];

					//qDebug()<<x<<y<<temp_angle_index<<mj;
					if (mj> m_max[level_index][0])
					{
						m_max[level_index][0] = mj;
						myx[level_index][0] = xx;
						myy[level_index][0] = yy;
						myangle_index[level_index][0] = temp_angle_index;

					}
				}
			}
		}


		if(level_index < MaxLevel-2)
			continue;

		myx[level_index][1] = 0;
		myy[level_index][1] = 0;
		if(myx[level_index+1][1] == 0 || myy[level_index+1][1] == 0)
			continue;
		myangle_index[level_index][1] = 0;
		m_max[level_index][1] = 0;
		for(int yy = myy[level_index+1][1]*2-3; yy <= myy[level_index+1][1]*2+3; ++yy)
		{
			for (int xx = myx[level_index+1][1]*2-3; xx<=myx[level_index+1][1]*2+3; ++xx)
			{

				for (int angle_index = myangle_index[level_index+1][1]*2 - 2; angle_index <= myangle_index[level_index+1][1]*2 + 2; angle_index++)
				{
					vector<double> temp_score;
					temp_score.clear();
					temp_angle_index = angle_index;
					if (angle_index < 0)
						temp_angle_index = angle_index + angle_size[level_index];

					if (angle_index >= angle_size[level_index])
						temp_angle_index = angle_index - angle_size[level_index];
					//vector<Point> temp_point_with_zero_score;
					//temp_point_with_zero_score.clear();


					float mj = 0;
					DIRECTION temp_template_direction, temp_image_direction;
					LOCATION temp_template_location;
					for (int template_point_index = 0; template_point_index < template_point_count[level_index]; template_point_index++)
					{
						temp_template_direction = template_direction[level_index][temp_angle_index][template_point_index];
						temp_template_location = template_location[level_index][temp_angle_index][template_point_index];

						temp_x_index = temp_template_location.x + xx;
						temp_y_index = temp_template_location.y + yy;

						if(!(temp_x_index >=0 && temp_x_index < image_width[level_index] && temp_y_index>=0 && temp_y_index < image_height[level_index]))
							continue;


						//temp_image_direction = image_direction_map[level_index][temp_y_index][temp_x_index];
						temp_image_direction = image_direction_map[level_index][temp_y_index*nCols+temp_x_index];


						//mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[ModelIndex][level_index][template_point_index] * image_direction_length[level_index][temp_y_index][temp_x_index]);
						mj = mj + abs((temp_template_direction.x * temp_image_direction.x + temp_template_direction.y * temp_image_direction.y)* template_direction_length[level_index][template_point_index] * image_direction_length[level_index][temp_y_index*nCols+temp_x_index]);
					}
					mj = mj/template_point_count[level_index];

					//qDebug()<<x<<y<<temp_angle_index<<mj;
					if (mj> m_max[level_index][1])
					{
						m_max[level_index][1] = mj;
						myx[level_index][1] = xx;
						myy[level_index][1] = yy;
						myangle_index[level_index][1] = temp_angle_index;

					}
				}
			}
		}


		if(level_index == MaxLevel-2)
		{
			if(m_max[level_index][0] < m_max[level_index][1])
			{
				m_max[level_index][0] = m_max[level_index][1];
				myx[level_index][0] = myx[level_index][1];
				myy[level_index][0] = myy[level_index][1];
				myangle_index[level_index][0] = myangle_index[level_index][1];
			}
		}
	}


	final_angle = myangle_index[0][0]*step[0];
	if(final_angle > 180)
		final_angle = final_angle - 360;


	if(m_max[0][0] > score_threshold && final_angle > min_angle && final_angle < max_angle)
	{
		x = myx[0][0];
		y = myy[0][0];
		angle = final_angle;
		score = m_max[0][0];
		match_count++;
	}


	for(int level_index = 0;level_index < MaxLevel;level_index++)
	{

		delete[] image_direction_map[level_index];
		delete[] image_direction_length[level_index];
		delete[] myx[level_index];
		delete[] myy[level_index];
		delete[] myangle_index[level_index];
		delete[] m_max[level_index];

	}

	delete[] image_direction_map;
	delete[] image_direction_length;

	delete[] myx;
	delete[] myy;
	delete[] myangle_index;
	delete[] m_max;

	delete[] image;
	delete[] image_width;
	delete[] image_height;
	delete[] image_sobel_x;
	delete[] image_sobel_y;


	return match_count;

}


Matcher MatcherAdapter::matcher[] = {Matcher()};

int MatcherAdapter::CreateModel(unsigned char* model, int width, int height, int MaxLevel, int sobel_size, float step_length, double threshold1, double threshold2, int MatcherIndex)
{
	return matcher[MatcherIndex].CreateModel(model, width, height, MaxLevel, sobel_size, step_length, threshold1, threshold2);
}

int MatcherAdapter::MatchMulti(unsigned char* image, int width, int height, int MaxLevel, float min_angle, float max_angle, int max_instance, float score_threshold, float* x, float* y, float* angle, float* score, int MatcherIndex)
{
	return matcher[MatcherIndex].MatchMulti(image, width, height, MaxLevel, min_angle, max_angle, max_instance, score_threshold, x, y, angle, score);
}

int MatcherAdapter::Match(unsigned char* image, int width, int height, int MaxLevel, float min_angle, float max_angle, float score_threshold, float& x, float& y, float& angle, float& score, int MatcherIndex)
{
	return matcher[MatcherIndex].Match(image, width, height, MaxLevel, min_angle, max_angle, score_threshold, x, y, angle, score);
}