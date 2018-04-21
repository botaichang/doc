3.keyframe 关键帧
=============================================
这里有线程锁的概念，还不是很清楚这块

关键帧，和普通的Frame不一样，但是可以由Frame来构造
许多数据会被三个线程同时访问，所以用锁的地方很普遍



关键帧包含了地图，路标点，帧，关键帧数据库等类

::

  class Map;
  class MapPoint;
  class Frame;
  class KeyFrameDatabase;

设置Pose,得到Pose,Pose的逆矩阵，Get相机中心，Get双目相机中心，Get旋转矩阵，Get平移,计算BoW

::

    // Pose functions
    // 这里的set,get需要用到锁
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();


图优化相关的一些函数

Covisibility graph是不同关键帧之间共享的可见点。

添加连接connection，删除连接，更新连接，更新最好的共享可见点.

添加子节点child，删除子节点，得到子节点

添加路标点MapPoint，删除路标点，得到路标点，

LoopEdge,

关键点 keypoint

::

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b)
    {
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2)
    {
        return pKF1->mnId<pKF2->mnId;
    }


下面的变量只可以单线程访问

包含了keyframe的ID号，时间戳，Grid,local mapping的一些变量,回环的一些变量

相机补偿的参数，等等

::

   // The following variables are accesed from only 1 thread or never change (no mutex needed).
   public:
   
       // nNextID名字改为nLastID更合适，表示上一个KeyFrame的ID号
       static long unsigned int nNextId;
       // 在nNextID的基础上加1就得到了mnID，为当前KeyFrame的ID号
       long unsigned int mnId;
       // 每个KeyFrame基本属性是它是一个Frame，KeyFrame初始化的时候需要Frame，
       // mnFrameId记录了该KeyFrame是由哪个Frame初始化的
       const long unsigned int mnFrameId;
   
       const double mTimeStamp;
   
       // Grid (to speed up feature matching)
       // 和Frame类中的定义相同
       const int mnGridCols;
       const int mnGridRows;
       const float mfGridElementWidthInv;
       const float mfGridElementHeightInv;
   
       // Variables used by the tracking
       long unsigned int mnTrackReferenceForFrame;
       long unsigned int mnFuseTargetForKF;
   
       // Variables used by the local mapping
       long unsigned int mnBALocalForKF;
       long unsigned int mnBAFixedForKF;
   
       // Variables used by the keyframe database
       long unsigned int mnLoopQuery;
       int mnLoopWords;
       float mLoopScore;
       long unsigned int mnRelocQuery;
       int mnRelocWords;
       float mRelocScore;
   
       // Variables used by loop closing
       cv::Mat mTcwGBA;
       cv::Mat mTcwBefGBA;
       long unsigned int mnBAGlobalForKF;
   
       // Calibration parameters
       const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
   
       // Number of KeyPoints
       const int N;
   
       // KeyPoints, stereo coordinate and descriptors (all associated by an index)
       // 和Frame类中的定义相同
       const std::vector<cv::KeyPoint> mvKeys;
       const std::vector<cv::KeyPoint> mvKeysUn;
       const std::vector<float> mvuRight; // negative value for monocular points
       const std::vector<float> mvDepth; // negative value for monocular points
       const cv::Mat mDescriptors;
   
       //BoW
       DBoW2::BowVector mBowVec; ///< Vector of words to represent images
       DBoW2::FeatureVector mFeatVec; ///< Vector of nodes with indexes of local features
   
       // Pose relative to parent (this is computed when bad flag is activated)
       cv::Mat mTcp;
   
       // Scale
       const int mnScaleLevels;
       const float mfScaleFactor;
       const float mfLogScaleFactor;
       const std::vector<float> mvScaleFactors;// 尺度因子，scale^n，scale=1.2，n为层数
       const std::vector<float> mvLevelSigma2;// 尺度因子的平方
       const std::vector<float> mvInvLevelSigma2;
   
       // Image bounds and calibration
       const int mnMinX;
       const int mnMinY;
       const int mnMaxX;
       const int mnMaxY;
       const cv::Mat mK;
   
   
   
   
   
