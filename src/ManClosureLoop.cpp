#if 0
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "SpaCostFunction.h"
#include "StaticIMUCostFunction.h"
#include <stdio.h>
#include <vector>
#include <memory>
#include "TrjFile.h"
#include "PcapParse.h"
#include <vector>
#include <string>
#include "IMUFile.h"
#include "LoopInfoFile.h"
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

Eigen::Isometry3d GetLoopTranform(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, Eigen::Isometry3d guess)
{
	pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudfilter1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudfilter2(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
	voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
	voxel_grid_filter.setInputCloud(cloud1);
	voxel_grid_filter.filter(*cloudfilter1);
	cloud1->clear();
	for (int i = 0; i < cloudfilter1->size(); i++)
	{
		pcl::PointXYZI &p = cloudfilter1->points[i];
		//if (p.x*p.x+p.y*p.y+p.z*p.z > 25.0)
		{
			cloud1->push_back(p);
		}
	}

	voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
	voxel_grid_filter.setInputCloud(cloud2);
	voxel_grid_filter.filter(*cloudfilter2);

	cloud2->clear();
	for (int i = 0; i < cloudfilter2->size(); i++)
	{
		pcl::PointXYZI &p = cloudfilter2->points[i];
		//if (p.x*p.x + p.y*p.y + p.z*p.z > 25.0)
		{
			cloud2->push_back(p);
		}
	}
	ndt.setInputSource(cloud2);
	ndt.setInputTarget(cloud1);
	ndt.setMaximumIterations(200);
	ndt.setResolution(2.0);
	ndt.setStepSize(1.0);
	ndt.setTransformationEpsilon(0.0001);

	pcl::PointCloud<pcl::PointXYZI> unused_result;
	ndt.align(unused_result, guess.matrix().cast<float>());
	pcl::io::savePCDFileBinary("cloud2-1.pcd", unused_result);
	pcl::io::savePCDFileBinary("cloud1.pcd", *cloud1);
	Eigen::Matrix4f mat = ndt.getFinalTransformation();
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
	Eigen::Quaterniond q;
	q = mat.cast<double>().block<3, 3>(0, 0);
	tf = q;
	tf(0, 3) = mat(0, 3);
	tf(1, 3) = mat(1, 3);
	tf(2, 3) = mat(2, 3);
	printf("%d\n", ndt.getFinalNumIteration());
	return tf;
}

Eigen::Isometry3d MakeTranform(Pose &pos1, Pose &pos2)
{
	Eigen::Isometry3d tf1 = GetTranform(pos1);
	Eigen::Isometry3d tf2 = GetTranform(pos2);
	return tf1.inverse()*tf2;
}


void Optimization(std::vector<Constraint> &constraints, std::vector<Pose> &vpos, std::vector<LoopInfo> &staticPoses)
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

	for (int i = 0; i < staticPoses.size(); i++)
	{
		LoopInfo &loop = staticPoses[i];
		CostFunction *cost_func = new AutoDiffCostFunction<StaticIMUCostFunction, 1, 4>(
			new StaticIMUCostFunction(loop.acc[0], loop.acc[1], loop.acc[2], 0.5));
		problem.AddResidualBlock(cost_func, new ceres::HuberLoss(10),
			vpos[loop.posidx].q);
	}

	problem.SetParameterBlockConstant(vpos[0].q);
	problem.SetParameterBlockConstant(vpos[0].tranlation);

	Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.num_threads = 8;
	options.max_num_iterations = 50;
	options.minimizer_progress_to_stdout = false;

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
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudi(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudj(new pcl::PointCloud<pcl::PointXYZI>);
	Pose pose;
	float d[4];
	char filename[255];
	char inputfilepath[255] = "H:/robot_data/立得厂房";
	int idx;
	uint64_t timestamp;
	std::vector<Pose> vpos;
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	Eigen::Isometry3d ismat = Eigen::Isometry3d::Identity();
	const int num = 2045;
	sprintf(filename, "%s/pose.txt", inputfilepath);
	FILE *fp = fopen(filename, "r");
	while(!feof(fp))
	{
		fscanf(fp, "%d %llu", &idx, &timestamp);
		fscanf(fp, "%f %f %f ", d, d + 1, d + 2);
		pose.tranlation[0] = d[0];
		pose.tranlation[1] = d[1];
		pose.tranlation[2] = d[2];
		fscanf(fp, "%f %f %f %f\n", d, d + 1, d + 2, d + 3);
		pose.q[0] = d[3];
		pose.q[1] = d[0];
		pose.q[2] = d[1];
		pose.q[3] = d[2];
		vpos.push_back(pose);
		printf("%d\n", idx);
	}
	fclose(fp);

	LoopInfoFile loopfile;
	std::vector<LoopInfo> staticPoses;
	std::map<unsigned int, std::vector<LoopInfo> > loopInfoMap;
	sprintf(filename, "%s/loop.txt", inputfilepath);
	loopfile.Open(filename);
	loopfile.GetData(loopInfoMap);
	printf("%d\n", loopInfoMap.size());
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
	std::map<unsigned int, std::vector<LoopInfo> >::iterator mapiter = loopInfoMap.begin();
	for (; mapiter != loopInfoMap.end(); ++mapiter)
	{
		for (int i = 0;i < mapiter->second.size();i++)
		{
			staticPoses.push_back(mapiter->second[i]);
			constraint.iIdx = mapiter->second[i].posidx;
			sprintf(filename, "%s/%d.pcd", inputfilepath, constraint.iIdx);
			printf("%s\n", filename);
			pcl::io::loadPCDFile(filename, *cloudi);
			for (int j = i+1; j < mapiter->second.size(); j++)
			{			
				constraint.jIdx = mapiter->second[j].posidx;			
				sprintf(filename, "%s/%d.pcd", inputfilepath, constraint.jIdx);
				printf("%s\n", filename);
				pcl::io::loadPCDFile(filename, *cloudj);
				constraint.zbar_ij = GetLoopTranform(cloudi, cloudj, MakeTranform(vpos[constraint.iIdx], vpos[constraint.jIdx]));
				constraint.scalar = 1;
				constraints.push_back(constraint);
			}
		}
	}

	std::vector<Pose> optimizePos;
	optimizePos = vpos;
	Optimization(constraints, optimizePos, staticPoses);
	SaveTraj("optimizeTraj.txt", optimizePos);

	for (int i = 0; i < optimizePos.size(); i++)
	{
		sprintf(filename, "%s/%d.pcd", inputfilepath, i);
		printf("%s\n", filename);
		pcl::io::loadPCDFile(filename, *cloud);
		pose = optimizePos[i];
		Eigen::Quaterniond q(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
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