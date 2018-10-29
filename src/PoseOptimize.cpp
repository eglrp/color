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
	unsigned long long fileDataIdx;
	unsigned char laserId;
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

class HeightResidual
{
public:
	HeightResidual(double height):height_(height)
	{
	}

	template <typename T> bool operator()(const T* const x0, T* residual) const
	{
		residual[0] = (x0[2] - T(height_))*0.1;
		return true;
	}
private:
	double height_;
};

void Optimization(std::vector<Constraint> &constraints, std::vector<Pose> &vpos)
{
	Problem problem;
	int cNum = constraints.size();
	for (int i = 0;i < vpos.size();i++)
	{
		problem.AddParameterBlock(vpos[i].tranlation, 3);
		std::unique_ptr<ceres::LocalParameterization> rotation_parametrization(new ceres::QuaternionParameterization);
		problem.AddParameterBlock(vpos[i].q, 4, rotation_parametrization.release());
	}
	for (int i = 0;i < cNum;i++)
	{
		Constraint &contraint = constraints[i];
		CostFunction *cost_func = new AutoDiffCostFunction<SpaCostFunction, 6, 4, 3, 4, 3>(
			new SpaCostFunction(contraint));
		problem.AddResidualBlock(cost_func, new ceres::HuberLoss(10),
			vpos[contraint.iIdx].q,vpos[contraint.iIdx].tranlation,
			vpos[contraint.jIdx].q,vpos[contraint.jIdx].tranlation);
	}

	for (int i = 1; i < vpos.size(); i++)
	{
		CostFunction *cost_func = new AutoDiffCostFunction<HeightResidual, 1, 3>(
			new HeightResidual(vpos[0].tranlation[2]));
		problem.AddResidualBlock(cost_func, new ceres::HuberLoss(100),
			vpos[i].tranlation);
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
	if(!fp)
		return -1;

	int num = vpos.size();
	for (int i = 0;i < num;i++)
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
	PcapParse pcap;
	TrjFileParse trj;
	TrjFile opttrj;
	opttrj.open("H:\\robot_data\\20180409154044\\opttrj-1.txt");
	uint64_t veltime, trjtime;
	Eigen::Isometry3d tran;
	pcap.OpenPcapFile("H:\\robot_data\\20180409154044\\0.pcap");
	//trj.open("H:\\robot_data\\20180409154044\\opttrj-1.txt");
	trj.open("H:\\robot_data\\20180409154044\\opt-trj.txt");
	HeaderTrj header;
	BlockTrj trjline;
	trj.readHeader(header);
	std::vector<std::string> vLaserName;
	for (size_t i = 0; i < header.laserNum; i++)
	{
		vLaserName.push_back(header.vLaserName[i]);
	}
	opttrj.writeHeader(vLaserName);

	trj.readBlock(trjline);
	trjtime = trjline.time;
	pcap.ReadPcapData(*cloud, veltime);
	int count = -1;
	int id = 0;
	const int submapnum = 37 * 20;
	char filename[255];
	Eigen::Isometry3d mat;

	Pose pos;
	std::vector<Pose> vpos;
	vpos.reserve(2000* submapnum);
	while (1)
	{
		if (trjtime < veltime)
		{
			int ret = trj.readBlock(trjline);
			if (ret != 0)
				break;
			trjtime = trjline.time;
			pos.timeStamp = trjtime;
			pos.fileDataIdx = trjline.fileDataIdx;
			pos.laserId = trjline.laserId;
			pos.q[0] = trjline.q[0];
			pos.q[1] = trjline.q[1];
			pos.q[2] = trjline.q[2];
			pos.q[3] = trjline.q[3];
			pos.tranlation[0] = trjline.pos[0];
			pos.tranlation[1] = trjline.pos[1];
			pos.tranlation[2] = trjline.pos[2];
			vpos.push_back(pos);
		}
		else if ((trjtime > veltime))
		{
			int ret = pcap.ReadPcapData(*cloud, veltime);
			if (ret != 0)
				break;
		}

		if (trjtime == veltime)
		{
			Eigen::Quaterniond q(trjline.q[0], trjline.q[1], trjline.q[2], trjline.q[3]);
			tran = q;
			tran(0, 3) = trjline.pos[0];
			tran(1, 3) = trjline.pos[1];
			tran(2, 3) = trjline.pos[2];
			pcl::transformPointCloud(*cloud, *cloud, tran.matrix());
			*out += *cloud;
			count++;
			if (count == submapnum)
			{
				//sprintf(filename, "H:\\robot_data\\20180409154044/1/%d.pcd", id++);
				//pcl::io::savePCDFileBinary(filename, *out);
				out->clear();
				count = 0;
			}
			int ret = pcap.ReadPcapData(*cloud, veltime);
			if (ret != 0)
				break;
		}
	}

	int nodesNum = vpos.size();
	Constraint constraint;

	std::vector<Constraint> constraints;
	constraints.reserve(4000 * submapnum);
	for (int i = 0;i < nodesNum-1;i++)
	{
		constraint.iIdx = i;
		constraint.jIdx = i+1;
		constraint.zbar_ij = MakeTranform(vpos[constraint.iIdx], vpos[constraint.jIdx]);
		constraint.scalar = 1;
		constraints.push_back(constraint);
	}

	for (int i = 0;i < nodesNum-4;i++)
	{
		constraint.iIdx = i;
		constraint.jIdx = i+4;
		constraint.zbar_ij = MakeTranform(vpos[constraint.iIdx], vpos[constraint.jIdx]);
		constraint.scalar = 1;
		constraints.push_back(constraint);
	}

	constraint.iIdx = 10* submapnum;
	constraint.jIdx = 1854 * submapnum;
	Eigen::Isometry3d tf1 = GetTranform(vpos[constraint.iIdx]);
	Eigen::Isometry3d tf2 = GetTranform(vpos[constraint.jIdx]);
	Eigen::Matrix4d mat1;
	mat1 << 0.998734951019, - 0.050282362849, - 0.000546460564, 3.439359188080,
		0.049981839955, 0.993841052055, - 0.098898112774, 3.285972356796,
		0.005515932105, 0.098745711148, 0.995096623898, - 25.509002685547,
		0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
	tf2.matrix() = mat1*tf2.matrix();
	constraint.zbar_ij = tf1.inverse()*tf2;
	constraint.scalar = 1;
	constraints.push_back(constraint);

	constraint.iIdx = 380 * submapnum;
	constraint.jIdx = 1770 * submapnum;
	tf1 = GetTranform(vpos[constraint.iIdx]);
	tf2 = GetTranform(vpos[constraint.jIdx]);

	mat1 << 0.993914365768, - 0.042505696416, - 0.101616621017, 5.569071292877,
		0.038809258491, 0.998520672321, - 0.038081265986, 1.418112277985,
		0.103084936738, 0.033905878663, 0.994094252586, - 14.535373687744,
		0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
	tf2.matrix() = mat1*tf2.matrix();
	constraint.zbar_ij = tf1.inverse()*tf2;
	constraint.scalar = 1;
	constraints.push_back(constraint);

	constraint.iIdx = 1231 * submapnum;
	constraint.jIdx = 1580 * submapnum;
	tf1 = GetTranform(vpos[constraint.iIdx]);
	tf2 = GetTranform(vpos[constraint.jIdx]);

	mat1 << 0.997380793095, -0.000108552842, -0.072336286306, 2.229337692261,
		0.002303195186, 0.999539971352, 0.030256673694, -0.034871991724,
		0.072299726307, - 0.030344020575, 0.996920883656, 4.138544559479,
		0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
	tf2.matrix() = mat1*tf2.matrix();
	constraint.zbar_ij = tf1.inverse()*tf2;
	constraint.scalar = 1;
	constraints.push_back(constraint);

	constraint.iIdx = 809 * submapnum;
	constraint.jIdx = 1639 * submapnum;
	tf1 = GetTranform(vpos[constraint.iIdx]);
	tf2 = GetTranform(vpos[constraint.jIdx]);

	mat1 << 0.997231543064, - 0.021433452144, - 0.071202129126, 4.029814720154,
		0.022256657481, 0.999694228172, 0.010788323358, - 0.090329281986,
		0.070949099958, - 0.012343179435, 0.997403562069, - 4.966828346252,
		0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
	tf2.matrix() = mat1*tf2.matrix();
	constraint.zbar_ij = tf1.inverse()*tf2;
	constraint.scalar = 1;
	constraints.push_back(constraint);

	std::vector<Pose> optimizePos;
	optimizePos = vpos;
	Optimization(constraints, optimizePos);
	SaveTraj("H:\\robot_data\\20180409154044/optimizeTraj.txt", optimizePos);
	for (int i = 0; i < optimizePos.size(); i++)
	{
		Pose &pose = optimizePos[i];
		trjline.time = pose.timeStamp;
		trjline.laserId = pose.laserId;
		trjline.fileDataIdx = pose.fileDataIdx;
		trjline.pos[0] = pose.tranlation[0];
		trjline.pos[1] = pose.tranlation[1];
		trjline.pos[2] = pose.tranlation[2];

		trjline.q[0] = pose.q[0];
		trjline.q[1] = pose.q[1];
		trjline.q[2] = pose.q[2];
		trjline.q[3] = pose.q[3];
		opttrj.writeBlock(trjline);
	}
	opttrj.close();
	return 0;
}
#endif