#pragma once
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cstdint>

#ifdef _OPENMP
#include <omp.h>
#endif

struct Point3D
{
    float x, y, z;
};

struct PointCloudAdaptor
{
    const std::vector<Point3D> &pts;

    PointCloudAdaptor(const std::vector<Point3D> &points) : pts(points) {}

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        if (dim == 1)
            return pts[idx].y;
        return pts[idx].z;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX &) const { return false; }
};

class FastNormalEstimator
{
public:
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
        PointCloudAdaptor,
        3>;

    FastNormalEstimator(float radius) : radius_(radius) {}

    void setInputCloud(const std::vector<Point3D> &cloud)
    {
        cloud_ = &cloud;
        adaptor_ = std::make_shared<PointCloudAdaptor>(cloud);
        kdtree_ = std::make_shared<KDTree>(
            3, *adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree_->buildIndex();
    }

    void compute(std::vector<Eigen::Vector3f> &normals_out)
    {
        normals_out.resize(cloud_->size());

        const float radius_sq = radius_ * radius_;

#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < static_cast<int>(cloud_->size()); ++i)
        {
            const Point3D &p = (*cloud_)[i];

            nanoflann::SearchParameters params;
            using Result = nanoflann::ResultItem<unsigned int, float>;
            std::vector<Result> ret_matches;

            size_t nMatches = kdtree_->radiusSearch(
                &p.x, radius_sq, ret_matches, params);

            if (nMatches < 5)
            {
                normals_out[i] = Eigen::Vector3f(0, 0, 1);
                continue;
            }

            // --- centroid ---
            Eigen::Vector3f mean(0, 0, 0);
            for (const auto &m : ret_matches)
            {
                const Point3D &q = (*cloud_)[m.first]; // <--- pakai first
                mean += Eigen::Vector3f(q.x, q.y, q.z);
            }
            mean /= static_cast<float>(nMatches);

            // --- covariance ---
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (const auto &m : ret_matches)
            {
                const Point3D &q = (*cloud_)[m.first]; // <--- pakai first
                Eigen::Vector3f d(q.x - mean.x(), q.y - mean.y(), q.z - mean.z());
                cov += d * d.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
            Eigen::Vector3f normal = solver.eigenvectors().col(0);
            normals_out[i] = normal.normalized();
        }
    }

private:
    float radius_;
    const std::vector<Point3D> *cloud_ = nullptr;
    std::shared_ptr<PointCloudAdaptor> adaptor_;
    std::shared_ptr<KDTree> kdtree_;
};
