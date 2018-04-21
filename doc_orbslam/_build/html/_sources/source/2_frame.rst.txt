2.Frame帧类
=============================================

Frame类中包含了MapPoint类和KeyFrame类

::

 #include "MapPoint.h"
 #include "Thirdparty/DBoW2/DBoW2/BowVector.h"
 #include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
 #include "ORBVocabulary.h"
 #include "KeyFrame.h"
 #include "ORBextractor.h"

 class MapPoint;
 class KeyFrame;


分别为双目摄像头，深度摄像头，单目摄像头三类构建帧类的复制构造函数

::

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);


抽取ORB特征

::

    // Extract ORB on the image. 0 for left image and 1 for right image.
    // 提取的关键点存放在mvKeys和mDescriptors中
    // ORB是直接调orbExtractor提取的
    void ExtractORB(int flag, const cv::Mat &im);

计算词袋BoW 

::

    // Compute Bag of Words representation.
    // 存放在mBowVec中
    void ComputeBoW();

设置相机位姿

::

    // Set the camera pose.
    // 用Tcw更新mTcw
    void SetPose(cv::Mat Tcw);

从相机姿态中计算旋转，平移和相机中心矩阵

::

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

得到相机中心点

::

    // Returns the camera center.
    inline cv::Mat GetCameraCenter()
    {
        return mOw.clone();
    }
    // Returns inverse of rotation


得到旋转矩阵的逆矩阵

::

    inline cv::Mat GetRotationInverse()
    {
        return mRwc.clone();
    }

判断路标点是否在视野中

::

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    // 判断路标点是否在视野中
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

判断关键点是否在grid中

::

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;


判断左右图关键点是否match,如果match,计算深度信息并将左右关键点坐标存储

::

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

将一个关键点从映射到3D世界坐标

::

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

