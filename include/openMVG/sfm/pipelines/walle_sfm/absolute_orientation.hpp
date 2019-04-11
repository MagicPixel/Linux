

#ifndef SFM_ABSOLUTE_ORIENTATION_H
#define SFM_ABSOLUTE_ORIENTATION_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

using namespace openMVG;
using namespace openMVG::sfm;

#define  RANSAC_PROB_BAD_SUPP 0.05
#define  RANSAC_ERR_TOL 5

namespace openMVG {
	namespace sfm {

		struct ResidualErrorFunctor_AO
		{
			ResidualErrorFunctor_AO(const double* const src_pt3d, const double* const dst_pt3d)
			{

				src_point3d[0] = src_pt3d[0];
				src_point3d[1] = src_pt3d[1];
				src_point3d[2] = src_pt3d[2];

				dst_point3d[0] = dst_pt3d[0];
				dst_point3d[1] = dst_pt3d[1];
				dst_point3d[2] = dst_pt3d[2];
			}

			template <typename T>
			bool operator()(
				const T* const params,
				T* out_residuals) const
			{

				//Vec3 X;
				const T rotation_angle[3] = { T(params[0]), T(params[1]), T(params[2]) };
				const T translation[3] = { T(params[3]), T(params[4]), T(params[5]) };
				const T scale = T(params[6]);

				T X[3];
				X[0] = T(src_point3d[0]);
				X[1] = T(src_point3d[1]);
				X[2] = T(src_point3d[2]);
				//旋转
				ceres::AngleAxisRotatePoint(rotation_angle, X, X);

				//平移
				X[0] = scale *  X[0] + translation[0];
				X[1] = scale *  X[1] + translation[1];
				X[2] = scale *  X[2] + translation[2];
				//std::cout << "transfom point :" << X[0] << "\t" << X[1] << "\t" << X[2] << std::endl;

				//Mat3 R;
				//ceres::AngleAxisToRotationMatrix((const double*)rotationAngle,
				//	R.data());



				//out_residuals[1] = T(X[1]) - T(dst_point3d[1]);					   T(X[0]) - T(dst_point3d[0])
				//out_residuals[2] = T(X[2]) - T(dst_point3d[2]);
				//std::cout << "[" << src_point3d[0] << "," << src_point3d[1] << "," << src_point3d[2] << "\t" << dst_point3d[0] << "," << dst_point3d[1] << "," << dst_point3d[2] << "]" << std::endl;

				//X[0] = src_point3d[0];
				//X[1] = src_point3d[1];
				//X[2] = src_point3d[2];
				//std::cout << "src point : " << src_point3d[0] << "\t" << src_point3d[1] << "\t" << src_point3d[2] << std::endl;
				//std::cout << "rotation :" << std::endl;
				//std::cout << rotationAngle[0] << "\t" << rotationAngle[1] << "\t" << rotationAngle[2] << std::endl;
				//std::cout << R(0, 0) << "\t" << R(0, 1) << "\t" << R(0, 2) << std::endl;
				//std::cout << R(1, 0) << "\t" << R(1, 1) << "\t" << R(1, 2) << std::endl;
				//std::cout << R(2, 0) << "\t" << R(2, 1) << "\t" << R(2, 2) << std::endl;

				//X = R * X ;

				//std::cout << "dst point : " << X[0] << "\t" << X[1] << "\t" << X[2] << std::endl;

				// Rotate the point according the camera rotation

				//ceres::AngleAxisRotatePoint(rotaion_angle, (T*)(src_point3d), X);

				//T _X[3] = { T(0), T(0), T(0)};
				// Apply the camera translation
				//_X[0] = (T(X[0]) + T(translation[0]));// * T(_scale);
				//_X[1] = (T(X[1]) + T(translation[1]));// * T(_scale);
				//[2] = (T(X[2]) + T(translation[2]));// * T(_scale);

				// Compute and return the error is the difference between the predicted
				//  and observed position


				//out_residuals[0] = T(_X[0]) - T(dst_point3d[0]);
				//out_residuals[1] = T(_X[1]) - T(dst_point3d[1]);
				//out_residuals[2] = T(_X[2]) - T(dst_point3d[2]);

				//std::cout << "residuals :" <<  g_t++ << std::endl;
				//std::cout << "scale :" << _scale << std::endl;
				//std::cout << T(_X[0]) - T(dst_point3d[0]) << std::endl;

				T rms_x = (T(X[0]) - T(dst_point3d[0])) * (T(X[0]) - T(dst_point3d[0]));
				T rms_y = (T(X[1]) - T(dst_point3d[1])) * (T(X[1]) - T(dst_point3d[1]));
				T rms_z = (T(X[2]) - T(dst_point3d[2])) * (T(X[2]) - T(dst_point3d[2]));

				//std::cout << "rms :" << rms_x << "\t " << rms_y << "\t" << rms_z << std::endl;
				out_residuals[0] = (rms_x + rms_y + rms_z);
				//std::cout << "residuals: " << T(out_residuals[0]) << std::endl;
				return true;
			}
			double src_point3d[3];
			double dst_point3d[3]; // The 3D observation
		};

		struct ResidualErrorFunctor_AO_Rotation
		{
			ResidualErrorFunctor_AO_Rotation(const double* const src_pt3d, const double* const dst_pt3d)
			{
				src_point3d[0] = src_pt3d[0];
				src_point3d[1] = src_pt3d[1];
				src_point3d[2] = src_pt3d[2];

				dst_point3d[0] = dst_pt3d[0];
				dst_point3d[1] = dst_pt3d[1];
				dst_point3d[2] = dst_pt3d[2];
			}

			template <typename T>
			bool operator()(
				const T* const params,
				T* out_residuals) const
			{

				//Vec3 X;
				const T rotation_angle[3] = { T(params[0]), T(params[1]), T(params[2]) };

				T X[3];
				X[0] = T(src_point3d[0]);
				X[1] = T(src_point3d[1]);
				X[2] = T(src_point3d[2]);
				//旋转
				ceres::AngleAxisRotatePoint(rotation_angle, X, X);
				T rms_x = (T(X[0]) - T(dst_point3d[0])) * (T(X[0]) - T(dst_point3d[0]));
				T rms_y = (T(X[1]) - T(dst_point3d[1])) * (T(X[1]) - T(dst_point3d[1]));
				T rms_z = (T(X[2]) - T(dst_point3d[2])) * (T(X[2]) - T(dst_point3d[2]));

				//std::cout << "rms :" << rms_x << "\t " << rms_y << "\t" << rms_z << std::endl;
				out_residuals[0] = (rms_x + rms_y + rms_z);
				//std::cout << "residuals: " << T(out_residuals[0]) << std::endl;
				return true;
			}

			double src_point3d[3];
			double dst_point3d[3]; // The 3D observation
		};

		struct ResidualErrorFunctor_AO_Translation
		{
			ResidualErrorFunctor_AO_Translation(const double* const src_pt3d, const double* const dst_pt3d)
			{
				src_point3d[0] = src_pt3d[0];
				src_point3d[1] = src_pt3d[1];
				src_point3d[2] = src_pt3d[2];

				dst_point3d[0] = dst_pt3d[0];
				dst_point3d[1] = dst_pt3d[1];
				dst_point3d[2] = dst_pt3d[2];
			}

			template <typename T>
			bool operator()(
				const T* const params,
				T* out_residuals) const
			{

				//Vec3 X;
				const T translation[3] = { T(params[0]), T(params[1]), T(params[2]) };

				T X[3];
				X[0] = T(src_point3d[0]) + translation[0];
				X[1] = T(src_point3d[1]) + translation[1];
				X[2] = T(src_point3d[2]) + translation[2];

				T rms_x = (T(X[0]) - T(dst_point3d[0])) * (T(X[0]) - T(dst_point3d[0]));
				T rms_y = (T(X[1]) - T(dst_point3d[1])) * (T(X[1]) - T(dst_point3d[1]));
				T rms_z = (T(X[2]) - T(dst_point3d[2])) * (T(X[2]) - T(dst_point3d[2]));

				//std::cout << "rms :" << rms_x << "\t " << rms_y << "\t" << rms_z << std::endl;
				out_residuals[0] = (rms_x + rms_y + rms_z);
				//std::cout << "residuals: " << T(out_residuals[0]) << std::endl;
				return true;
			}

			double src_point3d[3];
			double dst_point3d[3]; // The 3D observation
		};

		static ceres::CostFunction * CostFunctionAO(const double src[3], const double dst[3])
		{
			//return NULL;
			return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_AO, 1, 7>(
				new ResidualErrorFunctor_AO(src, dst));
		}

		static ceres::CostFunction * CostFunctionR(const double src[3], const double dst[3])
		{
			//return NULL;
			return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_AO_Rotation, 1, 3>(
				new ResidualErrorFunctor_AO_Rotation(src, dst));
		}

		static ceres::CostFunction * CostFunctionT(const double src[3], const double dst[3])
		{
			//return NULL;
			return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_AO_Translation, 1, 3>(
				new ResidualErrorFunctor_AO_Translation(src, dst));
		}

		class Ransac
		{
		public:
			Ransac();
			virtual ~Ransac();

			virtual bool Calculate(std::vector<std::size_t>& samples) = 0;

			virtual bool BundleAdjust() = 0;

			virtual std::size_t SampleSize() = 0;

			virtual std::size_t MinSampleSize() = 0;

			virtual double SampleRms(std::size_t sample) = 0;

			virtual std::vector<double> Params() = 0;

			
			std::vector<std::size_t> Inliners();

			std::vector<std::size_t> Outliners();

			bool Run();

			bool RunRms();

			double Rms();


		protected:

			

			void get_sample(std::vector<std::size_t>& samples);

			std::size_t find_inliners(const double max_rms);

		protected:

			//记录每一拟合的内点索引号
			std::vector<std::size_t> _inliners;
			//记录每一拟合的外点索引号
			std::vector<std::size_t> _outliners;

			//记录最佳内点集合索引
			std::vector<std::size_t> _best_inlines;

			std::vector<std::size_t> _sampled;				//size = _sample_num标志样本是否选取
			std::vector<std::size_t> _selected;				//size = m 选中的样本索引

			std::vector<double> _params;
			std::size_t _sample_num;						//参与拟合的全部样本数量
		};

		class  AbsoluteOrientation : public Ransac
		{
		public:
			typedef std::vector<Vec3> Points3f;
			typedef std::vector<Vec2> Points2f;
			typedef Mat3 Mat33;

			typedef geometry::Pose3 Pose;

			virtual bool Calculate(std::vector<std::size_t>& samples);

			virtual bool BundleAdjust();

			virtual std::size_t SampleSize();

			virtual std::size_t MinSampleSize();

			virtual double SampleRms(std::size_t sample);

			virtual std::vector<double> Params();

			void SetFixed(bool rotation_fixed, bool translation_fixed, bool scale_fixed);

			bool Process(SfM_Data& sfm_data, bool rotation_fixed, bool translation_fixed, bool scale_fixed);

			bool ProcessControl(SfM_Data& sfm_data, bool rotation_fixed, bool translation_fixed, bool scale_fixed);

			bool ProcessGNSS(SfM_Data& sfm_data, bool rotation_fixed, bool translation_fixed, bool scale_fixed);

			bool ProcessRansac(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, Mat3& R, Vec3& t, double& s);

			bool ProcessRotationT(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, Mat3& R, Vec3& t);
												  
			bool Process(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, Mat3& R, Vec3& t, double& s);

			void RotatePoints(const Points3f& src, Points3f& dst, Mat3& R);

			void control_relative_rms(const char* filepath, const Points3f& src, const Points3f& dst);

			Vec3 Transform(const Vec3& p, const Mat3& R, const Vec3& t, const double scale);

			Vec3 scale_point(const Vec3& p, const Vec3& c, const double& s);

			bool absolute_orientation(const Points3f& src, const Points3f& dst,
				Mat33& R, Vec3& t, double& s);

			bool calulate(const Points3f& src, const Points3f& dst,Mat33& R, Vec3& t);

			bool calulate(const Points3f& src, const Points3f& dst, Mat33& R, Vec3& t, double& s);

			bool calculate(const Vec3& a, const Vec3& b, const Vec3& c, 
				const Vec3& A, const Vec3& B, const Vec3& C, Mat33& R, Vec3& t, double& s);

			bool calculate(const Vec3& a, const Vec3& b, const Vec3& A, const Vec3& B,  
				Mat33& R, Vec3& t, double& s);

			bool is_line(const Vec3& a, const Vec3& b, const Vec3& c);

			bool is_obtuse_angle(const Vec3& a, const Vec3& b);

			bool is_obtuse_trangulate(const Vec3& a, const Vec3& b, const Vec3& c);

			bool absolute_orientation2(const Points3f& src, const Points3f& dst,
				Mat33& R, Vec3& t, double& s);

			void project_xoy(const Points3f& src, Points2f& dst);

			void project_xoz(const Points3f& src, Points2f& dst);

			void project_yoz(const Points3f& src, Points2f& dst);

			double calculate_rms(const Points3f& src, const Points3f& dst,
				const Mat33& R, const Vec3& t, const double& s, Vec3& rms);

			bool bundle_adjust(const Points3f& src, const Points3f& dst,
				Mat33& R, Vec3& t, double& s);

			bool bundle_adjust(const Points3f& src, const Points3f& dst,
				Vec3& t);

			bool bundle_adjust(const Points3f& src, const Points3f& dst,
				Mat33& R);

			Vec3 calculate_center(const Points3f& src, Points3f& dst);

			Mat3 trangle_rotation(const Vec3& A, const Vec3& B, const Vec3& C);

			Mat3 make_rotation(const Vec3& a, const Vec3& b);

			double trangle_area(const Vec3& A, const Vec3& B, const Vec3& C);

			void get_max_trangle(const Points3f& pts, int& a, int& b, int& c);

			Vec2 calculate_rotation(const Points2f& src, const Points2f& dst);

			void to_params(const Mat3& R, const Vec3& t, const double& s, std::vector<double>& params);

			void from_params(const std::vector<double>& params, Mat3& R, Vec3& t, double& s);

		protected:
			Vec3 _src_origin;
			Vec3 _dst_origin;

			Points3f _src;
			Points3f _dst;
			double _scale;
			Vec3 _translation;
			Mat3 _rotation;

			bool _rotation_fixed;
			bool _translation_fixed;
			bool _scale_fixed;
		};

	}
}
#endif // SFM_ABSOLUTE_ORIENTATION_H