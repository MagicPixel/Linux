
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#ifndef _WALLE_DATA_IO_HPP
#define _WALLE_DATA_IO_HPP

#include "openMVG/sfm/sfm_data.hpp"

#include "openMVG/stl/split.hpp"

#include "openMVG/sfm/sfm.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include<iomanip>
using namespace openMVG::cameras;

namespace openMVG { namespace sfm {

typedef Hash_Map<std::string, IndexT>  ImageNames;

struct feat2d_t
{
	double x;
	double y;
	IndexT class_id;
	IndexT seg_id;
};

inline std::ostream& operator<<(std::ostream& os, const feat2d_t& f)
{
	return  os << f.x << "\t" << f.y << "\t" << f.class_id << "\t" << f.seg_id << std::endl;
}

inline std::istream& operator>>(std::istream& is, feat2d_t& f)
{
	return is >> f.x >> f.y >> f.class_id >> f.seg_id;
}

struct feat3d_t
{
	double x;
	double y;
	double z;
	IndexT track_id;
	IndexT image_id;
	IndexT feat_id;
	IndexT class_id;
	IndexT seg_id;
};

inline std::ostream& operator<<(std::ostream& os, const feat3d_t& f)
{
	return  os << f.x << "\t" << f.y << "\t" << f.z << "\t" 
		<< f.track_id << "\t" << f.image_id << "\t" << f.feat_id << "\t" 
		<< f.class_id << "\t" << f.seg_id << std::endl;
}

inline std::istream& operator>>(std::istream& is, feat3d_t& f)
{
	return is >> f.x >> f.y >> f.z >> f.track_id >> f.image_id 
		>> f.feat_id >> f.class_id >> f.seg_id;
}

//with overloaded operators:

typedef std::vector<feat2d_t> Feats;

typedef Hash_Map<IndexT, Feats> Features;

struct Sfm_Block
{
	IndexT block_id;

	std::set<IndexT> images;
	std::set<IndexT> tracks;
};
typedef Hash_Map<IndexT, Sfm_Block >	Sfm_Blocks;

class WalleIO
{
public:
	WalleIO()
	{

	}

	~WalleIO()
	{

	}
	
	void Set(SfM_Data& sfm_data)
	{
		_sfm_data = sfm_data;
	}

	void Get(SfM_Data& sfm_data)
	{
		sfm_data = _sfm_data;
	}

	std::vector<SfM_Data> Blocks()
	{
		std::vector<SfM_Data> blocks;
		for (Sfm_Blocks::iterator it = _blocks.begin(); it != _blocks.end(); ++it)
		{
			blocks.push_back( FromBlock(it->second));
		}
		return blocks;
	}

	SfM_Data FromBlock(const Sfm_Block& block)
	{
		SfM_Data sfm_data;
		sfm_data.s_root_path = _sfm_data.s_root_path;
		sfm_data.block_id = block.block_id;

		//intrinsic
		for (std::set<IndexT>::iterator it = block.images.begin(); it != block.images.end(); ++it)
		{
			IndexT intrinsic_id = _sfm_data.views[*it]->id_intrinsic;
			if (_sfm_data.intrinsics.find(intrinsic_id) != _sfm_data.intrinsics.end())
			{
				sfm_data.intrinsics[intrinsic_id] = _sfm_data.intrinsics[intrinsic_id];
			}

		}

		//views
		for (std::set<IndexT>::iterator it = block.images.begin(); it != block.images.end(); ++it)
		{
			IndexT view_id = _sfm_data.views[*it]->id_view;
			if (_sfm_data.views.find(view_id) != _sfm_data.views.end())
			{
				sfm_data.views[view_id] = _sfm_data.views[view_id];
			}
		}

		//poses
		for (std::set<IndexT>::iterator it = block.images.begin(); it != block.images.end(); ++it)
		{
			IndexT pose_id = _sfm_data.views[*it]->id_pose;
			if (_sfm_data.poses.find(pose_id) != _sfm_data.poses.end())
			{
				sfm_data.poses[pose_id] = _sfm_data.poses[pose_id];
			}
		}

		//gnss
		for (std::set<IndexT>::iterator it = block.images.begin(); it != block.images.end(); ++it)
		{
			IndexT view_id = _sfm_data.views[*it]->id_view;

			if (_sfm_data.gps.find(view_id) != _sfm_data.gps.end())
			{
				sfm_data.gps[view_id] = _sfm_data.gps[view_id];
			}
		}

		//landmark
		for (std::set<IndexT>::iterator it = block.tracks.begin(); it != block.tracks.end(); ++it)
		{
			IndexT track_id = *it;
			if (_sfm_data.structure.find(track_id) != _sfm_data.structure.end())
			{
				sfm_data.structure[track_id] = _sfm_data.structure[track_id];
			}
		}
		return sfm_data;
	}

	bool CreateAll(const std::string& root, ESfM_Data flags_part)
	{
		_sfm_data.s_root_path = root;
		bool ret = true;
		const bool b_intrinsics = (flags_part & INTRINSICS) == INTRINSICS;
		
		if (b_intrinsics)
		{
			ret = ret && ImportCameras();
		}

		const bool b_views = (flags_part & VIEWS) == VIEWS;
		//影像和相机参数必须
		if (b_views)
		{
			ret = ret && ImportImages();
		}
		
		const bool b_extrinsics = (flags_part & EXTRINSICS) == EXTRINSICS;

		if (b_extrinsics)
		{
			ret = ret && ImportPoses();
		}

		const bool b_structure = (flags_part & STRUCTURE) == STRUCTURE;

		if (b_structure)
		{
			ret = ret && ImportLandMark();
		}
		const bool b_control_point = (flags_part & CONTROL_POINTS) == CONTROL_POINTS;

		if (b_control_point)
		{
			//ret = ret && ImportControl();
			std::cout << "Import control ok" << std::endl;
		}

		//const bool b_gnss = true; (flags_part & WITH_GNSS) == WITH_GNSS;
		//if (b_gnss)
		{
			ret = ret && ImportGNSS(_sfm_data);
		}

		//std::cout << "save ..." << std::endl;
		//return Save(
		//	_sfm_data,
		//	stlplus::create_filespec(_sfm_data.s_root_path, "all.json").c_str(),
		//	ESfM_Data(ALL));
		return true;
	}

	bool ImportBlocks(const std::string& root, std::vector<SfM_Data>& blocks, ESfM_Data flags_part)
	{
		std::vector<std::string> files = stlplus::folder_files(root + "/blocks");
		std::size_t num = files.size();
		for (std::size_t i = 0; i < num; i++)
		{
			if (stlplus::extension_part(files[i]) == "json")
			{
				std::cout << files[i] << std::endl;
				SfM_Data sfm_data;
				Load(sfm_data, root + "/blocks/" + files[i], flags_part);
				ImportGNSS(sfm_data);
				blocks.push_back(sfm_data);
			}
		}
		return blocks.size() > 0;
	}

	bool ExportBlocks(const std::string& result_root, const std::vector<SfM_Data>& blocks, ESfM_Data flags_part)
	{
		char filename[80]; 
		bool ret = true;
		for (std::size_t i = 0; i < blocks.size(); i++)
		{
			sprintf(filename, "%08d", blocks[i].block_id);
			ret = ret && Save(blocks[i], result_root + "/" + std::string(filename) + ".json", flags_part) &&
			Save(blocks[i], result_root + "/" + std::string(filename) + ".ply", flags_part) &&

			ExportGNSS(blocks[i], result_root + "/" + std::string(filename) + "_gnss.txt");
			ExportPoses(blocks[i], result_root + "/" + std::string(filename) + "_pose.txt");
			//输出landmarker

		}
		return ret;
	}

	bool ExportBlocks(const std::vector<SfM_Data>& blocks, ESfM_Data flags_part)
	{
		for (std::size_t i = 0; i < blocks.size(); i++)
		{
			char filename[80];
			sprintf(filename, "%08d", blocks[i].block_id);
				
			Save(blocks[i], blocks[i].s_root_path + "/blocks/" + std::string(filename) + ".json", flags_part);
			Save(blocks[i], blocks[i].s_root_path + "/pcd/" + std::string(filename) + ".ply", flags_part);
			ExportLandmarkers(blocks[i], blocks[i].s_root_path + "/objects/" + std::string(filename) + ".txt");
		}
		return true;
	}

	bool Export(const std::string& out_root, ESfM_Data flags_part)
	{
		// Store SfM_Data views & intrinsic data
		//Save(_sfm_data,
		//	stlplus::create_filespec(out_root, "sfm_data.json").c_str(),
		//	flags_part);

		std::vector<SfM_Data> blocks = Blocks();
		return ExportBlocks(out_root, blocks, flags_part);
	}

	bool ImportImages()
	{
		std::cout << "load images ..." << std::endl;
		std::string image_root = _sfm_data.s_root_path + "/images";
		
		Views & views = _sfm_data.views;
		std::vector<std::string> image_folders = stlplus::folder_subdirectories(image_root);

		IndexT view_id = 0;
		for (std::size_t i = 0; i < _cameranames.size(); i++)
		{
			std::string image_folder = image_root + "/" + _cameranames[i];

			std::vector<std::string> vec_image = stlplus::folder_files(image_folder);


			std::sort(vec_image.begin(), vec_image.end());

			for (std::vector<std::string>::const_iterator iter_image = vec_image.begin();
				iter_image != vec_image.end();
				++iter_image)
			{
				std::string imagepath = "/images/" + image_folders[i] + "/" + *iter_image;
				View v(imagepath, view_id, i, view_id, _sfm_data.intrinsics[i]->w(), _sfm_data.intrinsics[i]->h());
				// Add the view to the sfm_container
				views[v.id_view] = std::make_shared<View>(v);
				view_id++;
			}
		}

		std::cout << "generate image index" << std::endl;
		_image_index = get_imagenames(_sfm_data);
		GroupSharedIntrinsics(_sfm_data);
		std::cout << "load images : " << _sfm_data.views.size() << std::endl;
		
		return _sfm_data.views.size() > 0;
	}

	bool ImportCameras()
	{
		std::cout << "load cameras ..." << std::endl;
		std::string camera_root = _sfm_data.s_root_path + "/calibration";
		std::string image_root = _sfm_data.s_root_path + "/images";
	
		_cameranames = stlplus::folder_subdirectories(image_root);
		
		std::sort(_cameranames.begin(), _cameranames.end());

		std::size_t num = _cameranames.size();
		std::cout << num << std::endl;

		for (std::size_t i = 0; i < num; i++)
		{
			std::string cam_file = camera_root + "/" + _cameranames[i] + ".txt";
			std::cout << cam_file << std::endl;
			std::vector<double> cam_params;
			import_camera(cam_file.c_str(), cam_params);
			std::cout << "intrinsic" << std::endl;
			for (std::size_t j = 0; j < 10; j++)
			{
				std::cout << cam_params[j] << std::endl;
			}
			std::shared_ptr<IntrinsicBase> intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
				(int(cam_params[0]), int(cam_params[1]), cam_params[2], cam_params[3], cam_params[4],
				cam_params[5], cam_params[6], cam_params[7], cam_params[8], cam_params[9]);
			std::cout << "intrinsic" << std::endl;
			std::cout << intrinsic->w() << std::endl;
			_sfm_data.intrinsics[i] = intrinsic;
		}
		std::cout << "load cameras : " << num << std::endl;
		return _cameranames.size() > 0;
	}

	bool ImportMatches()
	{
		std::vector<std::string> files = stlplus::folder_files(_sfm_data.s_root_path + "/matches");

		//std::cout << "match files :" << files.size() << std::endl;
		for (std::size_t i = 0; i < files.size(); i++)
		{
			std::string filename  = stlplus::basename_part(files[i]);
			std::vector<std::string> imagenames;
			stl::split(filename, "-", imagenames);



			IndexT id1 = _image_index[imagenames[0]];
			IndexT id2 = _image_index[imagenames[1]];

			std::string filepath = _sfm_data.s_root_path + "/matches/" + files[i];

			//std::cout << imagenames[0] << "\t" << imagenames[1] << "\t" << id1 << "\t" << std::endl;
			//std::cout << filepath << std::endl;
			IndMatches matches;

			if (import_match(filepath.c_str(), matches))
			{
				_matches[std::make_pair(id1, id2)] = matches;
			}
			
		}

		return _matches.size() > 0;
	}

	bool BuildTracks()
	{
		// Compute tracks from matches
		tracks::TracksBuilder tracksBuilder;

		// List of features matches for each couple of images

		std::cout << "\n" << "Track building" << std::endl;

		tracksBuilder.Build(_matches);
		std::cout << "\n" << "Track filtering" << std::endl;
		tracksBuilder.Filter();
		std::cout << "\n" << "Track filtering : min occurence" << std::endl;
		tracksBuilder.FilterPairWiseMinimumMatches(3);
		std::cout << "\n" << "Track export to internal struct" << std::endl;
		//-- Build tracks with STL compliant type :
		tracksBuilder.ExportToSTL(_map_tracks);

		std::cout << "\n" << "Track stats" << std::endl;
		{
			std::ostringstream osTrack;
			//-- Display stats :
			//    - number of images
			//    - number of tracks
			std::set<size_t> set_imagesId;
			tracks::TracksUtilsMap::ImageIdInTracks(_map_tracks, set_imagesId);
			osTrack << "------------------" << "\n"
				<< "-- Tracks Stats --" << "\n"
				<< " Tracks number: " << tracksBuilder.NbTracks() << "\n"
				<< " Images Id: " << "\n";
			std::copy(set_imagesId.begin(),
				set_imagesId.end(),
				std::ostream_iterator<size_t>(osTrack, ", "));
			osTrack << "\n------------------" << "\n";

			std::map<size_t, size_t> map_Occurence_TrackLength;
			tracks::TracksUtilsMap::TracksLength(_map_tracks, map_Occurence_TrackLength);
			osTrack << "TrackLength, Occurrence" << "\n";
			for (std::map<size_t, size_t>::const_iterator iter =
				map_Occurence_TrackLength.begin();
				iter != map_Occurence_TrackLength.end(); ++iter)
			{
				osTrack << "\t" << iter->first << "\t" << iter->second << "\n";
			}
			osTrack << "\n";
			std::cout << osTrack.str();
		}

		std::cout << "track num : " << _map_tracks.size() << std::endl;

		for (openMVG::tracks::STLMAPTracks::iterator pos = _map_tracks.begin();
			pos != _map_tracks.end(); ++pos)
		{
			size_t trackid = pos->first;
			for (openMVG::tracks::submapTrack::iterator spos = pos->second.begin();
				spos != pos->second.end(); ++spos)
			{
				size_t imageid = spos->first;
				size_t featid = spos->second;
				_imageFeat_tracks[std::make_pair(imageid, featid)] = trackid;
			}
		}

		return _map_tracks.size() > 0;
	}

	bool ImportControl(Landmarks& landmarks)
	{
		_image_index = get_imagenames(_sfm_data);
		std::cout << "load control points ..." << std::endl;
		std::string filepath = _sfm_data.s_root_path + "/control.txt";
		std::ifstream ifstr(filepath.c_str());
		if (!ifstr.is_open())
		{
			return false;
		}
		
		std::size_t num = 0;
		ifstr >> num;
		
		Vec3 X;
		Vec2 x;
		std::string imagename;
		for (std::size_t i = 0; i < num; i++)
		{
			std::size_t view_num = 0;
			ifstr >> X[0] >> X[1] >> X[2] >> view_num;
		
			landmarks[i].X = X;
			for (std::size_t j = 0; j < view_num; j++)
			{
				ifstr >> imagename >> x[0] >> x[1];
				IndexT imageid = _image_index[imagename];
				//std::cout << imageid << std::endl;
				landmarks[i].obs[imageid].x = x;
			}
		}
		ifstr.close();
		//std::cout << "load control points : " << landmarks.size() << std::endl;
		return true;
	}

	bool ImportPoses()
	{
		std::cout << "load extrinsics ..." << std::endl;
		std::vector<IndexT> valid_views = get_valid_views();
		std::size_t num = valid_views.size();
		for (std::size_t i = 0; i < num; i++)
		{
			std::string imagename = get_imagename(i);
			std::string filepath = _sfm_data.s_root_path + "/poses/" + imagename + ".pose";
			geometry::Pose3 pose;
			import_pose(filepath.c_str(), pose);

			IndexT id_pose = _sfm_data.views[i]->id_pose;
			_sfm_data.poses[id_pose] = pose;
		}
		std::cout << "load extrinsics :" << num << std::endl;
		return num > 0;
	}

	bool ImportLandMark()			  
	{
		std::cout << "load landmark ..." << std::endl;
		if (!ImportMatches())
		{
			return false;
		}
	
		BuildTracks();

		std::vector<IndexT> valid_views = get_valid_views();
		for (std::size_t i = 0; i < valid_views.size(); i++)
		{
			import_landmark(i);
		}

		std::cout << "load landmark :" << _sfm_data.structure.size() << std::endl;
		return _sfm_data.structure.size() > 0;
	}

	bool ExportLandMark()
	{

	}
	  
	bool ImportGNSS(SfM_Data& sfm_data)
	{
		std::cout << "load gnss ..." << std::endl;
		for (Views::iterator it = sfm_data.views.begin(); it != sfm_data.views.end(); ++it)
		{
			std::string imagename = get_imagename(it->second);
			std::string filepath = sfm_data.s_root_path + "/gnss/" + imagename + ".txt";
			geometry::Pose3 pose;
			import_pose(filepath.c_str(), pose);

			IndexT id_pose = it->second->id_pose;
			sfm_data.gps[id_pose] = pose;
		}
		std::cout << "load gnss :" << sfm_data.gps.size() << std::endl;
		return sfm_data.gps.size() > 0;
	}

	bool ExportGNSS(const SfM_Data& sfm_data, const std::string& filename)
	{
		std::ofstream ofstr(filename.c_str());
		ofstr.setf(std::ios::fixed, std::ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
		ofstr.precision(3);
		if (!ofstr.is_open())
		{
			return false;
		}
		std::size_t num = sfm_data.gps.size();
		ofstr << num << std::endl;

		Poses poses = sfm_data.gps;
		Views views = sfm_data.views;
		for (Poses::iterator it = poses.begin(); it != poses.end(); ++it)
		{
			std::string imagename = get_imagename(views[it->first]);
			IndexT pose_id = views[it->first]->id_pose;
			Vec3 center = it->second.center();
			ofstr << imagename << "\t" << center[0] << "\t" << center[1] << "\t" << center[2] << "\t255\t0\t0" << std::endl;
		}

		ofstr.close();
		return true;
	}

	bool ExportPoses(const SfM_Data& sfm_data, const std::string& filename)
	{
		std::ofstream ofstr(filename.c_str());
		ofstr.setf(std::ios::fixed, std::ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
		ofstr.precision(3);
		if (!ofstr.is_open())
		{
			return false;
		}
		std::size_t num = sfm_data.poses.size();
		ofstr << num << std::endl;

		Poses poses = sfm_data.poses;
		Views views = sfm_data.views;
		for (Poses::iterator it = poses.begin(); it != poses.end(); ++it)
		{
			std::string imagename = get_imagename(views[it->first]);
			IndexT pose_id = views[it->first]->id_pose;
			Vec3 center = it->second.center();
			ofstr << imagename << "\t" << center[0] << "\t" << center[1] << "\t" << center[2] << "\t0\t255\t0" << std::endl;
		}

		ofstr.close();
		return true;
	}

	bool ExportLandmarkers(const SfM_Data& sfm_data, const std::string& filename)
	{
		// X	Y	Z	imagename	u	v	track_id	image_id	feat_id		class_id	seg_id

		//1. import Features

		Features feats;
		if (!import_feats(sfm_data, feats))
		{
			return false;
		}

		std::ofstream ofstr(filename.c_str());
		ofstr.setf(std::ios::fixed, std::ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
		ofstr.precision(3);
		if (!ofstr.is_open())
		{
			return false;
		}
		std::size_t num = 0;

		Landmarks structure = sfm_data.structure;
		for (Landmarks::iterator it = structure.begin(); it != structure.end(); ++it)
		{
			num += it->second.obs.size();
		}

		ofstr << num << "\t" << sfm_data.block_id << std::endl;

	
		for (Landmarks::iterator it = structure.begin(); it != structure.end(); ++it)
		{
			IndexT track_id = it->first;
			Vec3 X = it->second.X;
			Observations obs = it->second.obs;

			for (Observations::iterator o_it = obs.begin(); o_it != obs.end(); ++o_it)
			{
				IndexT image_id = o_it->first;
				std::string imagename = get_imagename(image_id);
				IndexT feat_id = o_it->second.id_feat;
				Vec2 x = o_it->second.x;

				IndexT class_id = feats[image_id][feat_id].class_id;
				IndexT seg_id = feats[image_id][feat_id].seg_id;

				ofstr << X[0] << "\t" << X[1] << "\t" << X[2] << "\t" 
					<< imagename << "\t" << x[0] << "\t" << x[1] << "\t"
					<< track_id << "\t" << image_id << "\t" << feat_id << "\t" 
					<< class_id << "\t" << seg_id << std::endl;
			}
		}

		ofstr.close();

		return  num > 0;
	}

protected:

	std::vector<IndexT>  get_valid_views()
	{
		std::vector<IndexT> valid_views;
		for (Views::const_iterator it = _sfm_data.GetViews().begin();
			it != _sfm_data.GetViews().end(); ++it)
		{
			const View * v = it->second.get();
			if (_sfm_data.GetIntrinsics().find(v->id_intrinsic) != _sfm_data.GetIntrinsics().end())
			{
				valid_views.push_back(v->id_view);
			}
		}
		return valid_views;
	}

	ImageNames get_imagenames(const SfM_Data& sfm_data)
	{
		ImageNames imagename_index;
		for (Views::const_iterator it = sfm_data.GetViews().begin();
			it != sfm_data.GetViews().end(); ++it)
		{
			IndexT imageid = it->second->id_view;
			std::string imagename = get_imagename(it->second);
			imagename_index[imagename] = imageid;
		}

		return imagename_index;
	}

	bool import_match(const IndexT& im1, const IndexT& im2)
	{
		std::string imagename1 = get_imagename(im1);
		std::string imagename2 = get_imagename(im2);

		std::string filepath = _sfm_data.s_root_path + "/matches/" + imagename1 + "_" + imagename2 + ".match";

		IndMatches matches;
	
		if (!import_match(filepath.c_str(), matches))
		{
			return false;
		}
		_matches[std::make_pair(im1, im2)] = matches;
		return matches.size() > 10;
	}

	bool import_match(const char* filepath, IndMatches& matches)
	{
		std::ifstream ifstr(filepath);
		if (!ifstr.is_open())
		{
			return false;
		}

		std::size_t num = 0;
		ifstr >> num;
		matches.resize(num);

		for (std::size_t i = 0; i < num; i++)
		{
			ifstr >> matches[i]._i >> matches[i]._j;
		}
	}

	std::string get_imagename(const std::shared_ptr<View> view)
	{
		return stlplus::basename_part(view->s_Img_path);
	}

	std::string get_imagename(const IndexT& imageid)
	{
		return get_imagename(_sfm_data.views[imageid]);
	}

	bool import_pose(const char* filepath, geometry::Pose3& pose)
	{
		std::ifstream ifstr(filepath);
		if (!ifstr.is_open())
		{
			return false;
		}

		Mat3 rotation;
		Vec3 c;

		ifstr >> c[0] >> c[1] >> c[2];
		ifstr >> rotation(0, 0) >> rotation(0, 1) >> rotation(0, 2);
		ifstr >> rotation(1, 0) >> rotation(1, 1) >> rotation(1, 2);
		ifstr >> rotation(2, 0) >> rotation(2, 1) >> rotation(2, 2);
		pose.center() = c;
		pose.rotation() = rotation;
		return true;
	}

	bool import_landmark(const IndexT imageid)
	{
		//std::cout << "Landmark :" << imageid << std::endl;
		if (_sfm_data.views.find(imageid) == _sfm_data.views.end())
		{
			std::cout << "can not find the image :" << imageid << std::endl;
			return false;
		}
		std::string imagename = get_imagename(imageid);
		std::string filepath = _sfm_data.s_root_path + "/landmark/" + imagename + ".txt";
		
		
		std::ifstream ifstr(filepath.c_str());
		if (!ifstr.is_open())
		{
			return false;
		}
		std::size_t num = 0, block_id = 0;
		ifstr >> num >> block_id;

		if (num == 0)
		{
			return false;
		}	

		_blocks[block_id].images.insert(imageid);

		_blocks[block_id].block_id = block_id;
		std::size_t trackid = 0, featid = 0;
		Vec3 X;
		Vec2 x;
		for (std::size_t i = 0; i < num; i++)
		{
			ifstr >> X[0] >> X[1] >> X[2] >> featid >> x[0] >> x[1];
			if (_imageFeat_tracks.find(std::make_pair(imageid, featid)) != _imageFeat_tracks.end())
			{
				trackid = _imageFeat_tracks[std::make_pair(imageid, featid)];
				_blocks[block_id].tracks.insert(trackid);
				_sfm_data.structure[trackid].X = X;
				_sfm_data.structure[trackid].obs[imageid] = Observation(x, featid);
				_sfm_data.structure[trackid].SetFixed(true);
			}
		}
		ifstr.close();
		return num > 0;
	}

	bool export_landmark(const char* filepath, Landmarks& landmarkers)
	{

	}

	bool import_camera(const char* filepath, std::vector<double>& params)
	{
		std::ifstream ifstr(filepath);
		if (!ifstr.is_open())
		{
			return false;
		}
		std::string str;
		params.resize(10);
		for (std::size_t i = 0; i < 10; i++)
		{
			ifstr >> str >> str >> params[i];
		}
		return true;
	}

	bool import_imagelist(const char* filepath, std::vector<std::string>& imagenames)
	{
		std::ifstream ifstr(filepath);
		if (!ifstr.is_open())
		{
			return false;
		}
		std::size_t num = 0;
		ifstr >> num;
		imagenames.resize(num);
		for (std::size_t i = 0; i < num; i++)
		{
			ifstr >> imagenames[i];
		}
		return true;
	}

	bool create_chunk(const std::string& chunk_root, SfM_Data& sfm_data)
	{
	
		sfm_data.s_root_path = _sfm_data.s_root_path;
		sfm_data.intrinsics = _sfm_data.intrinsics;

		Views & views = sfm_data.views;

		for (std::size_t i = 0; i < _cameranames.size(); i++)
		{
			std::string chunk_file = chunk_root + "/" + _cameranames[i] + ".chunk";

			std::vector<std::string> vec_image;
			import_imagelist(chunk_file.c_str(), vec_image);
	
			std::sort(vec_image.begin(), vec_image.end());

			for (std::vector<std::string>::const_iterator iter_image = vec_image.begin();
				iter_image != vec_image.end();
				++iter_image)
			{
				std::string imagepath = "/images/" + _cameranames[i] + "/" + *iter_image;

				View v(imagepath, views.size(), i, views.size(), _sfm_data.intrinsics[i]->w(), _sfm_data.intrinsics[i]->h());
				// Add the view to the sfm_container
				views[v.id_view] = std::make_shared<View>(v);
			}

		}

	}

	bool import_feats(const SfM_Data& sfm_data, Features& feats)
	{
		bool ret = true;
		for (Views::const_iterator it = sfm_data.GetViews().begin();
			it != sfm_data.GetViews().end(); ++it)
		{
			IndexT imageid = it->second->id_view;
			std::string imagename = get_imagename(it->second);
			std::string filepath = sfm_data.s_root_path + "/features/" + imagename + ".feat";
			ret = ret && import_feats(filepath.c_str(), feats[imageid]);
		}

		return ret;
	}

	bool import_feats(const char* filepath, Feats& feats)
	{
		feats.clear();
		std::ifstream ifstr(filepath);
		if (!ifstr.is_open())
		{
			return false;
		}

		std::copy(
			std::istream_iterator<Feats::value_type >(ifstr),
			std::istream_iterator<Feats::value_type >(),
			std::back_inserter(feats));
		bool bOk  = !ifstr.bad();
		ifstr.close();
		return bOk;
	}
protected:
	ImageNames _image_index;
	std::vector<std::string> _cameranames;
	openMVG::matching::PairWiseMatches _matches;
	openMVG::tracks::STLMAPTracks  _map_tracks;
	openMVG::tracks::ImageFeatTracks _imageFeat_tracks;

	SfM_Data _sfm_data;
	Sfm_Blocks _blocks;
	
};

}  } // namespace sfm   // namespace openMVG

#endif // _WALLE_DATA_IO_HPP
