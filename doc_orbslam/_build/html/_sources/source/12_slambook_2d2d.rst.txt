12_PoseEstimate 2d-2d
======================================================


主程序
---------------------------------------

- 读取两张图片
 
- 寻找特征匹配点
 
- pose_estimation_2d2d ( pts1, pts2, R, t )

::

   int main ( int argc, char** argv )
   {
       if ( argc != 3 )
       {
           cout<<"usage: pose_estimation_2d2d img1 img2"<<endl;
           return 1;
       }
       //-- 读取图像
       Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
       Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );
   
       vector<KeyPoint> keypoints_1, keypoints_2;
       vector<DMatch> matches;
       find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
       cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;
   
       //-- 估计两张图像间运动
       Mat R,t;
       pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );
   
       //-- 验证E=t^R*scale
       Mat t_x = ( Mat_<double> ( 3,3 ) <<
                   0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                   t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                   -t.at<double> ( 1.0 ),     t.at<double> ( 0,0 ),      0 );
   
       cout<<"t^R="<<endl<<t_x*R<<endl;
   
       //-- 验证对极约束
       Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
       for ( DMatch m: matches )
       {
           Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
           Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
           Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
           Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
           Mat d = y2.t() * t_x * R * y1;
           cout << "epipolar constraint = " << d << endl;
       }
       return 0;
   }
   

pose_estimation_2d2d/2d-2d如何获得R,T
------------------------------------------

1. 将匹配的特征点放入findFundamentalMat()函数，获得fundamental_matrix

2. 根据fundamental_matrix以及相机光心和焦距获得essential_matrix

3. 根据essential_matrix通过recoverPose函数获得相机位姿


::

  void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                              std::vector<KeyPoint> keypoints_2,
                              std::vector< DMatch > matches,
                              Mat& R, Mat& t )
  {
      // 相机内参,TUM Freiburg2
      Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
  
      //-- 把匹配点转换为vector<Point2f>的形式
      vector<Point2f> points1;
      vector<Point2f> points2;
  
      for ( int i = 0; i < ( int ) matches.size(); i++ )
      {
          points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
          points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
      }
  
      //-- 计算基础矩阵
      Mat fundamental_matrix;
      fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
      cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;
  
      //-- 计算本质矩阵
      Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
      double focal_length = 521;			//相机焦距, TUM dataset标定值
      Mat essential_matrix;
      essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
      cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;
  
      //-- 计算单应矩阵
      Mat homography_matrix;
      homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
      cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;
  
      //-- 从本质矩阵中恢复旋转和平移信息.
      recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
      cout<<"R is "<<endl<<R<<endl;
      cout<<"t is "<<endl<<t<<endl;
      
  }
