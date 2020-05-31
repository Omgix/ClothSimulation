#ifndef CLOTHSIMULATION_SPATIALHASHING
#define CLOTHSIMULATION_SPATIALHASHING

#include <Eigen/Dense>

#include <deque>
#include <vector>

namespace Simulator {
struct HashTableEntry {
	enum Type {
		CLOTH,
		RIGID,
	};
	Eigen::Vector3i cell;
	Type type;
	unsigned int object;
	unsigned int element;
};

class SpatialHashing {
private:
	typedef HashTableEntry value_type;
	typedef std::deque<value_type> buckle_type;
	typedef buckle_type::const_iterator entry_handle;

	unsigned int _buckle_size;
	std::vector<buckle_type> _buckles; // Fixed size array
	//std::vector<entry_handle> _end_iters;
	
	inline unsigned int hash(int ix, int iy, int iz) const {
		return ((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)) % _buckle_size;
	}

public:
	SpatialHashing(unsigned int buckle_size) :
		_buckles(buckle_size), _buckle_size(buckle_size)/*, _end_iters(buckle_size)*/ {
		_buckles.shrink_to_fit();
		//_end_iters.shrink_to_fit();
	}

	void pre_next_step() {
		//for (unsigned int i = 0; i < _buckle_size; ++i)
			//_end_iters[i] = _buckles[i].begin();
		for (buckle_type& buckle : _buckles)
			buckle.clear();
	}

	void clear() {
		for (buckle_type& buckle : _buckles)
			buckle.clear();
		//for (unsigned int i = 0; i < _buckle_size; ++i)
			//_end_iters[i] = _buckles[i].begin();
	}

	void insert(const Eigen::Vector3i& cell, HashTableEntry::Type type, unsigned int object, unsigned int element) {
		unsigned int key = hash(cell.x(), cell.y(), cell.z());
		_buckles[key].push_front(HashTableEntry{ cell, type, object, element });
	}

	void insert(int i, int j, int k, HashTableEntry::Type type, unsigned int object, unsigned int element) {
		unsigned int key = hash(i, j, k);
		_buckles[key].push_front(HashTableEntry{ Eigen::Vector3i(i, j, k), type, object, element });
	}

	void insert_range(int minx, int maxx, int miny, int maxy, int minz, int maxz, HashTableEntry::Type type, unsigned int object, unsigned int element) {
		for (int i = minx; i <= maxx; i++)
			for (int j = miny; j <= maxy; j++)
				for (int k = minz; k <= maxz; k++) {
					unsigned int key = hash(i, j, k);
					_buckles[key].push_front(HashTableEntry{ Eigen::Vector3i(i, j, k), type, object, element });
				}
	}

	template< class InputIt >
	void insert(InputIt first, InputIt last) {
		for (; first != last; first++) {
			unsigned int key = hash(first->cell.x(), first->cell.y(), first->cell.z());
			_buckles[key].push_front(*first);
		}
	}

	template < class OutputIt >
	OutputIt find_in_cell(const Eigen::Vector3i& cell, OutputIt& iterator) const {
		unsigned int key = hash(cell.x(), cell.y(), cell.z());
		auto copyFunction = [cell](const value_type& entry) { return entry.cell == cell; };
		entry_handle beginIt = _buckles[key].begin();
		entry_handle endIt = _buckles[key].end();//_end_iters[key];
		return std::copy_if(beginIt, endIt, iterator, copyFunction);
	}

	template < class OutputIt >
	OutputIt find_in_cell(int i, int j, int k, OutputIt& iterator) const {
		unsigned int key = hash(i, j, k);
		Eigen::Vector3i cell(i, j, k);
		auto copyFunction = [cell](const value_type& entry) { return entry.cell == cell; };
		entry_handle beginIt = _buckles[key].begin();
		entry_handle endIt = _buckles[key].end();//_end_iters[key];
		return std::copy_if(beginIt, endIt, iterator, copyFunction);
	}
};
}

#endif // !CLOTHSIMULATION_SPATIALHASHING