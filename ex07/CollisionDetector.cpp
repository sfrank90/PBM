#include "CollisionDetector.h"

void CollisionDetector::sort(std::vector<const EndPoint*> &endpoints, std::set<std::pair<const AABB*, const AABB*> > &overlaps) {
	/* Implement a simple in-place "bubble sort" algorithm for the endpoints here.
	 * Whenever a start point and an end point are swapped, the corresponding
	 * parent boxes start or cease to overlap. Update the set of overlaps accordingly
	 * by inserting or erasing pairs of overlapping bounding boxes. In order for the
	 * pairs to be uniquely defined, you need to maintain a consistent ordering for
	 * the two members of the pair. Use the function make_sorted_pair to create an
	 * appropriately ordered pair of bounding boxes.
	 */
	size_t n = endpoints.size();

    do {
        size_t newn = 1;
        for(size_t i = 0; i < n-1; ++i) {
            if(EndPoint::comp_ptr(endpoints[i+1], endpoints[i])) {
                const EndPoint *tmp = endpoints[i+1];
                endpoints[i+1] = endpoints[i];
                endpoints[i] = tmp;
                //start overlap
                if(endpoints[i]->is_start() && endpoints[i+1]->is_end() && endpoints[i]->parent() != endpoints[i+1]->parent()) {
                    overlaps.insert(make_sorted_pair(endpoints[i]->parent(), endpoints[i+1]->parent()));
                }
                //cease overlap
                if(endpoints[i+1]->is_start() && endpoints[i]->is_end() && endpoints[i]->parent() != endpoints[i+1]->parent()) {
                    overlaps.erase(make_sorted_pair(endpoints[i]->parent(), endpoints[i+1]->parent()));
                }
                newn += 1;
            }
        }
        n = newn;
    } while(n > 1);
}

std::vector<std::pair<const AABB*, const AABB*> > CollisionDetector::detect_possible_collisions() {
	std::vector<std::pair<const AABB*, const AABB*> > intersection;
	/* Implement an algorithm for detecting overlapping bounding boxes here.
	 * For each dimension (x, y, and z), there is a sorted list of bounding box
	 * start and end points stored in _endpoints. A set of bounding boxes whose
	 * coordinate ranges in this dimension overlap is stored in _overlaps.
	 * First, for each dimension, update the endpoints and overlaps using the
	 * sort function implemented above. Then, the set of possible collisions
	 * is simply the intersection of the sets of overlaps in all dimensions,
	 * that is, the set of pairs of bounding boxes whose coordinate ranges
	 * overlap in every dimension.
	 */
	for(size_t i = 0; i < _endpoints.size(); i++) {
        sort(_endpoints[i],_overlaps[i]);
    }
    for(std::set<std::pair<const AABB*, const AABB*> >::iterator it = _overlaps[0].begin(); it !=  _overlaps[0].end(); ++it) {
        bool b = true;
        for(size_t i = 0; i < _overlaps.size() && b; ++i) {
            b &= (_overlaps[i].find(*it) != _overlaps[i].end());
        }
        if(b)
            intersection.push_back(*it);
    }
	return intersection;
}

