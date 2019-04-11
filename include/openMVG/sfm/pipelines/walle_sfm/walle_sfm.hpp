
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/tracks/tracks.hpp"
#include "absolute_orientation.hpp"

namespace openMVG {
namespace sfm {

/// Sequential SfM Pipeline Reconstruction Engine.
class WalleEngine : public ReconstructionEngine
{
public:

	WalleEngine(
    const SfM_Data & sfm_data,
    const std::string & soutDirectory,
	const std::string & loggingFile = "");

	~WalleEngine();

  void SetFeaturesProvider(Features_Provider * provider);
  void SetMatchesProvider(Matches_Provider * provider);

  void SetBaseline(const double baseline)
  {
	  _baseline = baseline;
  }

  virtual bool Process();

  bool Process2();

  bool Process3();

  bool ProcessWithGPS();

  void setInitialPair(const Pair & initialPair)
  {
    _initialpair = initialPair;
  }

  /// Initialize tracks
  bool InitLandmarkTracks();

  /// Select a candidate initial pair
  bool ChooseInitialPair(Pair & initialPairIndex) const;

  /// Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
  bool MakeInitialPair3D(const Pair & initialPair);

  bool MakeInitialPair3DBaseline(const Pair & initialPair, const double baseline);

  bool MakeInitialPair3DRC(const Pair & initialPair, const Mat3& rotation, Vec3& center);

  bool MakeInitialPair3D2();

  bool ImportInitalPair3D();

  bool ImportLandMarks(const std::size_t& imageid);

  bool ImportLandMarks(const char* filepath);

  bool ImportPoses(const char* filepath);

  bool ImportPose(const std::size_t& imageid);

  bool ImportGPS(const std::size_t& imageid, Vec3& center, Mat3& rotation);

  bool ImportRotationCenter(const char* filepath, Vec3& center, Mat3& rotation);

  void ImportPoses();

  bool UpdateLandMarks();

  void UpdatePoses();

  void Interpolate();

  void FilterTracjectory();

  void UpdatePoses(const Mat3& R, const Vec3& t, const double s);

  void UpdateLandMarkers(const Mat3& R, const Vec3& t, const double s);

  /** !brief			根据基线更新比例尺
  
  \param	[in]		baseline			基线长度

  \return	void
  */
  void UpdatePosition(const double& baseline);

  /** !brief			根据平差结果，计算两个相机之间的距离

  \param	[in]		void

  \return	double
  */
  double CalculateBaseLine();

  double CalculateProjectErr(const Vec3& X, const Vec2& x, const Mat34& P);

  /// Automatic initial pair selection (based on a 'baseline' computation score)
  bool AutomaticInitialPairChoice(Pair & initialPair) const;

  /**
   * Set the default lens distortion type to use if it is declared unknown
   * in the intrinsics camera parameters by the previous steps.
   *
   * It can be declared unknown if the type cannot be deduced from the metadata.
   */
  void SetUnknownCameraType(const cameras::EINTRINSIC camType)
  {
    _camType = camType;
  }

  double ComputeResiduals();

  bool BundleAdjustment2();
protected:


	void absolute_orientation();

	bool is_continue(Vec3& a, Vec3& b, Vec3& c);

	Vec2 feat(const std::size_t& imageid, const std::size_t& featid);

	Vec2 undistort_feat(const std::size_t& imageid, const std::size_t& featid);

private:

	//bool read_pose(const char* filepath, Pose3& pose);

  /// List the images that the greatest number of matches to the current 3D reconstruction.
  bool FindImagesWithPossibleResection(std::vector<size_t> & vec_possible_indexes, 
	   const float dThresholdGroup = 0.75f);

  void ResectionPossibleImages(const float dThresholdGroup = 0.75f);

  /// Add a single Image to the scene and triangulate new possible tracks.
  bool Resection(const size_t imageIndex);

  /// Bundle adjustment to refine Structure; Motion and Intrinsics
  bool BundleAdjustment();



  /// Discard track with too large residual error
  size_t badTrackRejector(double dPrecision, size_t count = 0);

  //----
  //-- Data
  //----

  // HTML logger
  //std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
  std::string _sLoggingFile;

  // Parameter
  Pair _initialpair;
  cameras::EINTRINSIC _camType; // The camera type for the unknown cameras

  //-- Data provider
  Features_Provider  * _features_provider;
  Matches_Provider  * _matches_provider;

  // Temporary data
  openMVG::tracks::STLMAPTracks _map_tracks; // putative landmark tracks (visibility per 3D point)
  openMVG::tracks::ImageFeatTracks	 _imageFeat_tracks;
  
  std::vector<bool> _viewFixed;
  Hash_Map<IndexT, double> _map_ACThreshold; // Per camera confidence (A contrario estimated threshold error)
  C_Progress_display* _ptr_progress_bar;
  std::set<size_t> _set_remainingViewId;     // Remaining camera index that can be used for resection

  Hash_Map<std::string, IndexT> _block_map;

  Poses _gps;

  bool _bneed_orientation;

  double _baseline;
  Mat3 _rotation;
  Vec3 _center;
};
								  
} // namespace sfm
} // namespace openMVG

