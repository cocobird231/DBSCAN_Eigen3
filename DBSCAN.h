#pragma once
#include <Eigen/Dense>
#include <vector>

namespace DBSCAN
{
	enum PointType { None = -1, Core, Border, Noise };
	class DBPoint
	{
	public:
		Eigen::Vector3f pt;
		int ID;
		PointType ptType;
		std::vector<int> directPt;
		int ptClass;
		bool coreDirectlyReachable;
		DBPoint(Eigen::Vector3f pt, int ID) : pt(pt), ID(ID), ptType(PointType::None), ptClass(-1), coreDirectlyReachable(false)
		{
			directPt.reserve(100);
		}
	};

	void DBClusterIter(std::vector<DBPoint>& DBPoints, DBPoint& corePoint, int ParentClass)
	{
		for (auto& _DBPt : corePoint.directPt)
		{
			if (DBPoints[_DBPt].ptType == PointType::Border)// Clustering ignore border points
				continue;
			if (DBPoints[_DBPt].ptClass != ParentClass)
			{
				DBPoints[_DBPt].ptClass = ParentClass;
				DBClusterIter(DBPoints, DBPoints[_DBPt], ParentClass);
			}
		}
		return;
	}

	void DBSCAN(std::vector<Eigen::Vector3f>& rawPoints, std::vector<DBPoint>& corePoints, std::vector<DBPoint>& borderPoints, std::vector<DBPoint>& Noises, float eps, int MinPts)
	{
		int PCSize = rawPoints.size();
		std::vector<DBPoint> DBPoints;
		DBPoints.reserve(PCSize);
		for (int i = 0; i < PCSize; i++)
			DBPoints.push_back(DBPoint(rawPoints[i], i));
		// Find Core points
		for (int i = 0; i < PCSize; i++)
		{
			for (int j = 0; j < PCSize; j++)
			{
				if (i == j)
					continue;
				if ((DBPoints[i].pt - DBPoints[j].pt).norm() <= eps)
					DBPoints[i].directPt.push_back(j);
					
			}
			if (DBPoints[i].directPt.size() >= MinPts)
			{
				DBPoints[i].ptType = PointType::Core;
				for (auto& dptID : DBPoints[i].directPt)
					DBPoints[dptID].coreDirectlyReachable = true;
			}
		}
		// Find Border points and Noises
		for (auto& _DBPt : DBPoints)
		{
			if (_DBPt.ptType == PointType::Core)
				continue;
			if (_DBPt.coreDirectlyReachable)
				_DBPt.ptType = PointType::Border;
			else
				_DBPt.ptType = PointType::Noise;
		}
		// Start Clustering
		int classNum = 0;
		for (auto& _DBPt : DBPoints)
		{
			if (_DBPt.ptType == PointType::Core && _DBPt.ptClass == -1)
			{
				_DBPt.ptClass = classNum;
				DBClusterIter(DBPoints, _DBPt, classNum);
				classNum++;
			}
		}
		// Saving each type of points
		corePoints.clear();
		borderPoints.clear();
		Noises.clear();
		for (auto& _DBPt : DBPoints)
			if (_DBPt.ptType == PointType::Core)
				corePoints.push_back(_DBPt);
			else if (_DBPt.ptType == PointType::Border)
				borderPoints.push_back(_DBPt);
			else if (_DBPt.ptType == PointType::Noise)
				Noises.push_back(_DBPt);
	}

	void DBSCAN(std::vector<Eigen::Vector3f>& rawPoints, std::vector<std::vector<Eigen::Vector3f>>& corePoints, std::vector<Eigen::Vector3f>& borderPoints, std::vector<Eigen::Vector3f>& Noises, float eps, int MinPts)
	{
		std::vector<DBPoint> _corePoints;
		std::vector<DBPoint> _borderPoints;
		std::vector<DBPoint> _Noises;
		DBSCAN(rawPoints, _corePoints, _borderPoints, _Noises, eps, MinPts);
		int maxClass = 0;
		for (auto& _cp : _corePoints)
			maxClass = _cp.ptClass > maxClass ? _cp.ptClass : maxClass;
		corePoints = std::vector<std::vector<Eigen::Vector3f>>(maxClass + 1);
		borderPoints.clear();
		borderPoints.reserve(_borderPoints.size());
		Noises.clear();
		Noises.reserve(_Noises.size());
		for (auto& _cp : _corePoints)
			corePoints[_cp.ptClass].push_back(_cp.pt);
		for (auto& _bp : _borderPoints)
			borderPoints.push_back(_bp.pt);
		for (auto& _n : _Noises)
			Noises.push_back(_n.pt);
	}
}