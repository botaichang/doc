8.ORBmatcher ORB 特征匹配
======================================

特征匹配
----------------------------

计算ORB描述子的汉明距离

::

      // Computes the Hamming distance between two ORB descriptors
      static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);


Search matches between Frame keypoints and projected MapPoints. Returns number of matches


- 搜索关键点和投影地图点的匹配。返回匹配的个数

- 经常被用来跟踪局部地图


- @brief 通过投影，对Local MapPoint进行跟踪

- 将Local MapPoint投影到当前帧中, 由此增加当前帧的MapPoints \n

- 在SearchLocalPoints()中已经将Local MapPoints重投影（isInFrustum()）到当前帧 \n

- 并标记了这些点是否在当前帧的视野中，即mbTrackInView \n

- 对这些MapPoints，在其投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
 
- @param  F           当前帧
 
- @param  vpMapPoints Local MapPoints
 
- @param  th          阈值
 
- @return             成功匹配的数量
 
- @see SearchLocalPoints() isInFrustum()
       

Project MapPoints seen in KeyFrame into the Frame and search matches.

::

      int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);
  
Used in relocalisation (Tracking)

::

      int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);
  
  
Project MapPoints using a Similarity Transformation and search matches.
Used in loop detection (Loop Closing)

::
      int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);
  

Use for Loop Detection
----------------------

::

      rute force constrained to ORB that belong to the same vocabulary node (at a certain level)
      Used in Relocalisation and Loop Detection
      @brief 通过词包，对关键帧的特征点进行跟踪
      
      KeyFrame中包含了MapPoints，对这些MapPoints进行tracking \n
      由于每一个MapPoint对应有描述子，因此可以通过描述子距离进行跟踪 \n
      为了加速匹配过程，将关键帧和当前帧的描述子划分到特定层的nodes中 \n
      对属于同一node的描述子计算距离进行匹配 \n
      通过距离阈值、比例阈值和角度投票进行剔除误匹配
      @param  pKF               KeyFrame
      @param  F                 Current Frame
      @param  vpMapPointMatches F中MapPoints对应的匹配，NULL表示未匹配
      @return                   成功匹配的数量
      
      int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
      int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);
  

Matching for the Map Initialization (only used in the monocular case)

::

      int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);
  
Matching to triangulate new MapPoints. Check Epipolar Constraint.

::

      int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                                 std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);
  
Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]

::

      // In the stereo and RGB-D case, s12=1
      int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
  
Project MapPoints into KeyFrame and search for duplicated MapPoints.

::

      int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);
  
Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.

::

      int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);
  


others functions 
------------------------
::

  public:
  
      static const int TH_LOW;
      static const int TH_HIGH;
      static const int HISTO_LENGTH;
  
  
  protected:
  
      bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);
  
      float RadiusByViewingCos(const float &viewCos);
  
      void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);
  
      float mfNNratio;
      bool mbCheckOrientation;
  };
