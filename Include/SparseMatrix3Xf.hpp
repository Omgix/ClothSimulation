#ifndef CLOTHSIMULATION_SPARSEMATRIX3XF
#define CLOTHSIMULATION_SPARSEMATRIX3XF

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>

#include <vector>
#include <list>

class SparseMatrix3Xf;
using Eigen::SparseMatrix;

namespace Eigen {
namespace internal {
    // SparseMatrix3Xf looks-like a SparseMatrix, so let's inherits its traits:
    template<>
    struct traits<SparseMatrix3Xf> : public Eigen::internal::traits<Eigen::SparseMatrix<float> >
    {};
}
}

class SparseMatrix3Xf : public Eigen::EigenBase<SparseMatrix3Xf> {
public:
    typedef float Scalar;
    typedef float RealScalar;
    typedef int StorageIndex;
    enum {
        ColsAtCompileTime = Eigen::Dynamic,
        MaxColsAtCompileTime = Eigen::Dynamic,
        IsRowMajor = true
    };

    Index rows() const { return _size * 3; }
    Index cols() const { return _size * 3; }

    template<typename Rhs>
    Eigen::Product<SparseMatrix3Xf, Rhs, Eigen::AliasFreeProduct> operator*(const Eigen::MatrixBase<Rhs>& x) const {
        return Eigen::Product<SparseMatrix3Xf, Rhs, Eigen::AliasFreeProduct>(*this, x.derived());
    }

    SparseMatrix3Xf(int size) : _size(size), _mat(size) {}
    SparseMatrix3Xf(const SparseMatrix3Xf&& rhs) noexcept : _size(rhs._size), _mat(std::move(rhs._mat)){}
    SparseMatrix3Xf(const SparseMatrix3Xf& rhs) = delete;

	static SparseMatrix3Xf Identity(StorageIndex size) {
        SparseMatrix3Xf result(size);

        for (StorageIndex i = 0; i < size; ++i)
            result._mat[i].emplace_front(i, Eigen::Matrix3f::Identity());

        return result;
    }

    inline StorageIndex size() const {
        return _size;
    }

    Eigen::VectorXf multi(const Eigen::VectorXf& vec) const {
        Eigen::VectorXf result = Eigen::VectorXf::Zero(_size * 3);
        Eigen::MatrixXf::Identity(3,3);

        for (StorageIndex i = 0; i < _size; ++i)
	        for (const sub_mat& subMatrix: _mat[i])
	            result.segment<3>(i * 3) += subMatrix.value * vec.segment<3>(i * 3);

        return result;
    }

    SparseMatrix3Xf& multi_sub3_rows_matrix3f(StorageIndex sub3Row, Eigen::Matrix3f& lhs) {
        for (sub_mat& subMatrix : _mat[sub3Row])
		    subMatrix.value = lhs * subMatrix.value;
    	
        return *this;
    }

    Eigen::Matrix3f& sub_matrix(StorageIndex i, StorageIndex j) {
        assert(i < _size && j < _size);

        std::list<sub_mat>& sub3cols = _mat[i];

        for (auto it = sub3cols.begin(); it != sub3cols.end(); ++it) {
            const StorageIndex index = it->sub3col_index;
            if (index == j)
                return it->value;
            else if (index > j)
                return sub3cols.emplace(it, j, Eigen::Matrix3f::Zero())->value;
        }

        return sub3cols.emplace(sub3cols.end(), j, Eigen::Matrix3f::Zero())->value;
    }

    SparseMatrix3Xf& operator=(SparseMatrix3Xf&& rhs) noexcept {
        _size = rhs._size;
        _mat = std::move(rhs._mat);

        return *this;
    }

    SparseMatrix3Xf operator+(SparseMatrix3Xf& rhs) const {
        assert(_size == rhs._size);
        SparseMatrix3Xf result(_size);

        for (StorageIndex i = 0; i < _size; ++i) {
            auto it1 = _mat[i].begin();
            auto it2 = rhs._mat[i].begin();

            while (it1 != _mat[i].end() || it2 != rhs._mat[i].end())
                if (it1 == _mat[i].end()) {
                    result._mat[i].emplace(result._mat[i].end(), it2->sub3col_index, it2->value);
                    ++it2;
                } else if (it2 == rhs._mat[i].end()) {
                    result._mat[i].emplace(result._mat[i].end(), it1->sub3col_index, it1->value);
                    ++it1;
                } else if (it1->sub3col_index < it2->sub3col_index) {
                    result._mat[i].emplace(result._mat[i].end(), it1->sub3col_index, it1->value);
                    ++it1;
                } else if (it2->sub3col_index < it1->sub3col_index) {
                    result._mat[i].emplace(result._mat[i].end(), it2->sub3col_index, it2->value);
                    ++it2;
                } else if (it1->sub3col_index == it2->sub3col_index) {
                    result._mat[i].emplace(result._mat[i].end(), it1->sub3col_index, it1->value + it2->value);
                    ++it1;
                    ++it2;
                }
        }

        return result;
    }

    SparseMatrix3Xf& operator+=(SparseMatrix3Xf& rhs) {
        assert(_size == rhs._size);

        for (StorageIndex i = 0; i < _size; ++i) {
            auto it1 = _mat[i].begin();

            for (auto it2 = rhs._mat[i].begin(); it2 != rhs._mat[i].end(); ++it2) {
                while (it1 != _mat[i].end() && it1->sub3col_index < it2->sub3col_index)
                    ++it1;

                if (it1 == _mat[i].end() || it1->sub3col_index > it2->sub3col_index)
                    _mat[i].emplace(it1, it2->sub3col_index, it2->value);
                else if (it1->sub3col_index == it2->sub3col_index)
                    it1->value += it2->value;
            }
        }

        return *this;
    }

    SparseMatrix3Xf operator-(SparseMatrix3Xf& rhs) const {
        assert(_size == rhs._size);
        SparseMatrix3Xf result(_size);

        for (StorageIndex i = 0; i < _size; ++i) {
            auto it1 = _mat[i].begin();
            auto it2 = rhs._mat[i].begin();

            while (it1 != _mat[i].end() || it2 != rhs._mat[i].end())
                if (it1 == _mat[i].end()) {
                    result._mat[i].emplace(result._mat[i].end(), it2->sub3col_index, -it2->value);
                    ++it2;
                } else if (it2 == rhs._mat[i].end()) {
                    result._mat[i].emplace(result._mat[i].end(), it1->sub3col_index, it1->value);
                    ++it1;
                } else if (it1->sub3col_index < it2->sub3col_index) {
                    result._mat[i].emplace(result._mat[i].end(), it1->sub3col_index, it1->value);
                    ++it1;
                } else if (it2->sub3col_index < it1->sub3col_index) {
                    result._mat[i].emplace(result._mat[i].end(), it2->sub3col_index, -it2->value);
                    ++it2;
                } else if (it1->sub3col_index == it2->sub3col_index) {
                    result._mat[i].emplace(result._mat[i].end(), it1->sub3col_index, it1->value - it2->value);
                    ++it1;
                    ++it2;
                }
        }

        return result;
    }

    SparseMatrix3Xf& operator-=(SparseMatrix3Xf& rhs) {
        assert(_size == rhs._size);

        for (StorageIndex i = 0; i < _size; ++i) {
            auto it1 = _mat[i].begin();

            for (auto it2 = rhs._mat[i].begin(); it2 != rhs._mat[i].end(); ++it2) {
                while (it1 != _mat[i].end() && it1->sub3col_index < it2->sub3col_index)
                    ++it1;

                if (it1 == _mat[i].end() || it1->sub3col_index > it2->sub3col_index)
                    _mat[i].emplace(it1, it2->sub3col_index, -it2->value);
                else if (it1->sub3col_index == it2->sub3col_index)
                    it1->value -= it2->value;
            }
        }

        return *this;
    }

    SparseMatrix3Xf operator*(Scalar rhs) const {
        SparseMatrix3Xf result(_size);
        for (StorageIndex i = 0; i < result._size; ++i)
            for (sub_mat& mat : result._mat[i])
                mat.value *= rhs;

        return result;
    }

    SparseMatrix3Xf& operator*=(Scalar rhs) {
        for (StorageIndex i = 0; i < _size; ++i)
            for (sub_mat& mat : _mat[i])
                mat.value *= rhs;

        return *this;
    }
private:
    struct sub_mat {
        StorageIndex sub3col_index;
        Eigen::Matrix3f value;
        sub_mat(const StorageIndex& index, const Eigen::Matrix3f& mat) : sub3col_index(index), value(mat) {}
        sub_mat(const StorageIndex&& index, const Eigen::Matrix3f&& mat) : sub3col_index(index), value(mat) {}
        sub_mat(const sub_mat& rhs) : sub3col_index(rhs.sub3col_index), value(rhs.value) {}
        sub_mat(const sub_mat&& rhs) : sub3col_index(rhs.sub3col_index), value(std::move(rhs.value)) {}
    };
    StorageIndex _size;
    std::vector<std::list<sub_mat>> _mat;
};

namespace Eigen {
namespace internal {

template<typename Rhs>
struct generic_product_impl<SparseMatrix3Xf, Rhs, SparseShape, DenseShape, GemvProduct> // GEMV stands for matrix-vector
    : generic_product_impl_base<SparseMatrix3Xf, Rhs, generic_product_impl<SparseMatrix3Xf, Rhs> >
{
    typedef typename Product<SparseMatrix3Xf, Rhs>::Scalar Scalar;

    template<typename Dest>
    static void scaleAndAddTo(Dest& dst, const SparseMatrix3Xf& lhs, const Rhs& rhs, const Scalar& alpha)
    {
        // This method should implement "dst += alpha * lhs * rhs" inplace,
        // however, for iterative solvers, alpha is always equal to 1, so let's not bother about it.
        assert(alpha == Scalar(1) && "scaling is not implemented");
        EIGEN_ONLY_USED_FOR_DEBUG(alpha);

        // Here we could simply call dst.noalias() += lhs.my_matrix() * rhs,
        // but let's do something fancier (and less efficient):
        dst += lhs.multi(rhs);
        
    }

};


}
}

#endif // !CLOTHSIMULATION_SPARSEMATRIX3XF
