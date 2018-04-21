6.Map地图
==================================

地图负责管理关键帧，路标点的功能
----------------------------------

在地图中添加关键帧，添加路标点，删除路标点，删除关键帧，设置参考路标点，获得所有关键帧，过得参考的地图点

::

  class Map
  {
  public:
      Map();
  
      void AddKeyFrame(KeyFrame* pKF);
      void AddMapPoint(MapPoint* pMP);
      void EraseMapPoint(MapPoint* pMP);
      void EraseKeyFrame(KeyFrame* pKF);
      void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
  
      std::vector<KeyFrame*> GetAllKeyFrames();
      std::vector<MapPoint*> GetAllMapPoints();
      std::vector<MapPoint*> GetReferenceMapPoints();
  
      long unsigned int MapPointsInMap();
      long unsigned  KeyFramesInMap();
  
      long unsigned int GetMaxKFid();
  
      void clear();
  
      vector<KeyFrame*> mvpKeyFrameOrigins;
  
      std::mutex mMutexMapUpdate;
  
      // This avoid that two points are created simultaneously in separate threads (id conflict)
      std::mutex mMutexPointCreation;
  
  protected:
      std::set<MapPoint*> mspMapPoints; ///< MapPoints
      std::set<KeyFrame*> mspKeyFrames; ///< Keyframs
  
      std::vector<MapPoint*> mvpReferenceMapPoints;
  
      long unsigned int mnMaxKFid;
  
      std::mutex mMutexMap;
  };

