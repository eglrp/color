#if 0
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "SpaCostFunction.h"
#include <stdio.h>
#include <vector>
#include <memory>
#include "TrjFile.h"
#include "PcapParse.h"
#include <vector>
#include <string>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct Pose
{
	unsigned long long timeStamp;
	double tranlation[3];
	double q[4];
};
Eigen::Isometry3d GetTranform(Pose &pos)
{
	/*Eigen::AngleAxisd xrot(pos.rotation[0], Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yrot(pos.rotation[1], Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd zrot(pos.rotation[2], Eigen::Vector3d::UnitZ());*/
	Eigen::Quaterniond q(pos.q[0], pos.q[1], pos.q[2], pos.q[3]);
	Eigen::Isometry3d tf;
	tf = q;
	tf(0, 3) = pos.tranlation[0];
	tf(1, 3) = pos.tranlation[1];
	tf(2, 3) = pos.tranlation[2];
	return tf;
}

Eigen::Isometry3d MakeTranform(Pose &pos1, Pose &pos2)
{
	Eigen::Isometry3d tf1 = GetTranform(pos1);
	Eigen::Isometry3d tf2 = GetTranform(pos2);
	return tf1.inverse()*tf2;
}


void Optimization(std::vector<Constraint> &constraints, std::vector<Pose> &vpos)
{
	Problem problem;
	int cNum = constraints.size();
	for (int i = 0; i < vpos.size(); i++)
	{
		problem.AddParameterBlock(vpos[i].tranlation, 3);
		std::unique_ptr<ceres::LocalParameterization> rotation_parametrization(new ceres::QuaternionParameterization);
		problem.AddParameterBlock(vpos[i].q, 4, rotation_parametrization.release());
	}
	for (int i = 0; i < cNum; i++)
	{
		Constraint &contraint = constraints[i];
		CostFunction *cost_func = new AutoDiffCostFunction<SpaCostFunction, 6, 4, 3, 4, 3>(
			new SpaCostFunction(contraint));
		problem.AddResidualBlock(cost_func, new ceres::HuberLoss(10),
			vpos[contraint.iIdx].q, vpos[contraint.iIdx].tranlation,
			vpos[contraint.jIdx].q, vpos[contraint.jIdx].tranlation);
	}

	problem.SetParameterBlockConstant(vpos[0].q);
	problem.SetParameterBlockConstant(vpos[0].tranlation);

	Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.num_threads = 8;
	options.max_num_iterations = 50;
	options.minimizer_progress_to_stdout = true;

	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
}

int SaveTraj(const char *filename, std::vector<Pose> &vpos)
{
	FILE *fp = fopen(filename, "w");
	if (!fp)
		return -1;

	int num = vpos.size();
	for (int i = 0; i < num; i++)
	{
		Pose &pos = vpos[i];
		fprintf(fp, "%d %lf %lf %lf\n", pos.timeStamp,
			pos.tranlation[0],
			pos.tranlation[1],
			pos.tranlation[2]);
	}
	fclose(fp);
	return 0;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	Pose pose;
	float d[4];
	char filename[255];
	char inputfilepath[255] = "H:/robot_data/InterMedia20180411122057";
	int idx;
	std::vector<Pose> vpos;
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	Eigen::Isometry3d ismat = Eigen::Isometry3d::Identity();
	const int num = 2045;
	sprintf(filename, "%s/pose.txt", inputfilepath);
	FILE *fp = fopen(filename, "r");
	for (int i = 0; i < num; i++)
	{
		fscanf(fp, "%d ", &idx);
		for (int j = 0; j < 3; j++)
		{
			fscanf(fp, "%f %f %f %f ", d, d + 1, d + 2, d + 3);
			mat(j, 0) = d[0];
			mat(j, 1) = d[1];
			mat(j, 2) = d[2];
			mat(j, 3) = d[3];
		}
		fscanf(fp, "%f %f %f %f\n", d, d + 1, d + 2, d + 3);
		ismat.matrix() = mat;
		pose.timeStamp = i;
		pose.tranlation[0] = ismat.translation().x();
		pose.tranlation[1] = ismat.translation().y();
		pose.tranlation[2] = ismat.translation().z();
		Eigen::Quaterniond q;
		q = ismat.rotation();
		pose.q[0] = q.x();
		pose.q[1] = q.y();
		pose.q[2] = q.z();
		pose.q[3] = q.w();
		vpos.push_back(pose);
	}
	fclose(fp);

	int nodesNum = vpos.size();
	Constraint constraint;

	std::vector<Constraint> constraints;
	constraints.reserve(4000);
	for (int i = 0; i < nodesNum - 1; i++)
	{
		constraint.iIdx = i;
		constraint.jIdx = i + 1;
		constraint.zbar_ij = MakeTranform(vpos[constraint.iIdx], vpos[constraint.jIdx]);
		constraint.scalar = 1;
		constraints.push_back(constraint);
	}

	for (int i = 0; i < nodesNum - 4; i++)
	{
		constraint.iIdx = i;
		constraint.jIdx = i + 4;
		constraint.zbar_ij = MakeTranform(vpos[constraint.iIdx], vpos[constraint.jIdx]);
		constraint.scalar = 1;
		constraints.push_back(constraint);
	}

	/*constraint.iIdx = 10;
	constraint.jIdx = 1854;
	Eigen::Isometry3d tf1 = GetTranform(vpos[constraint.iIdx]);
	Eigen::Isometry3d tf2 = GetTranform(vpos[constraint.jIdx]);
	Eigen::Matrix4d mat1;
	mat1 << 0.998734951019, -0.050282362849, -0.000546460564, 3.439359188080,
		0.049981839955, 0.993841052055, -0.098898112774, 3.285972356796,
		0.005515932105, 0.098745711148, 0.995096623898, -25.509002685547,
		0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
	tf2.matrix() = mat1*tf2.matrix();
	constraint.zbar_ij = tf1.inverse()*tf2;
	constraint.scalar = 1;
	constraints.push_back(constraint);

	constraint.iIdx = 380;
	constraint.jIdx = 1770;
	tf1 = GetTranform(vpos[constraint.iIdx]);
	tf2 = GetTranform(vpos[constraint.jIdx]);

	mat1 << 0.993914365768, -0.042505696416, -0.101616621017, 5.569071292877,
		0.038809258491, 0.998520672321, -0.038081265986, 1.418112277985,
		0.103084936738, 0.033905878663, 0.994094252586, -14.535373687744,
		0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
	tf2.matrix() = mat1*tf2.matrix();
	constraint.zbar_ij = tf1.inverse()*tf2;
	constraint.scalar = 1;
	constraints.push_back(constraint);*/

	std::vector<Pose> optimizePos;
	optimizePos = vpos;
	Optimization(constraints, optimizePos);
	SaveTraj("optimizeTraj.txt", optimizePos);

	for (int i = 0; i < num; i++)
	{
		sprintf(filename, "%s/%06d.pcd", inputfilepath, i);
		printf("%s\n", filename);
		pcl::io::loadPCDFile(filename, *cloud);
		pose = optimizePos[i];
		Eigen::Quaterniond q(pose.q);
		ismat = q;
		ismat(0, 3) = pose.tranlation[0];
		ismat(1, 3) = pose.tranlation[1];
		ismat(2, 3) = pose.tranlation[2];
		pcl::transformPointCloud(*cloud, *cloud, ismat.matrix());
		sprintf(filename, "%s/1-%d.pcd", inputfilepath, i);
		pcl::io::savePCDFileBinary(filename, *cloud);
	}
	return 0;
}
#endif