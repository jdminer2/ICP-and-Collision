// Working on this.
// https://en.wikipedia.org/wiki/K-d_tree

import { medianFind } from "./utils.js";

/* 
    this.size: how many points are in this KDTree.
    this.point: the center root point of the KDTree.
    this.left: the left subtree, which is also a KDTree.
    this.right: the right subtree, which is also a KDTree.
    this.axis: the axis used for comparison at the top level of the tree, which is x, y, or z.
    this.comparison: returns the distance between a given point and the root point, using the root's axis. 
      Positive iff given point's axis position is greater than this.point's axis position.
    this.belongsLeft: returns true if a given point belongs left of the root point, using the root's axis.
*/
export class KDTree {
    // Depth is only supposed to be used internally. Externally, this should be called as just KDTree(points).
    constructor(points, depth = 0) {
        this.depth = depth;
        this.axis = ["x","y","z"][depth % 3];
        this.size = points.length;

        // If size is 0, then there is no center point or subtrees.
        if(this.size == 0) {
            this.point = null;
            this.left = null;
            this.right = null;
            return;
        }

        // center root point has the median value on this node's axis.
        let medianIndex = medianFind(points, (a,b) => a[this.axis] - b[this.axis]);
        this.point = points[medianIndex];

        // Sort points into the left or right list. 
        let leftArray = [];
        let rightArray = [];
        points.forEach((point,index) => {
            // Don't put the median point in the subtrees, but do put other tied points in.
            if(index != medianIndex) {
                // Push the point into the side that it belongs in.
                if(this.belongsLeft(point))
                    leftArray.push(point);
                else
                    rightArray.push(point);
            }
        })
        // Turn the left and right lists into subtrees.
        this.left = new KDTree(leftArray,depth+1);
        this.right = new KDTree(rightArray,depth+1);
    }

    // Null this.point returns 0.
    comparison(point) {
        if(!this.point)
            return 0;
        return point[this.axis] - this.point[this.axis];
    }

    // Ties or null this.point returns true.
    belongsLeft(point) {
        if(!this.point)
            return true;
        return point[this.axis] <= this.point[this.axis];
    }
    
    // Determines if the tree contains any point whose distance squared to the target is less than or equal to minSquaredDistance.
    searchExistence(target, minSquaredDistance = Infinity) {
        // If the KDTree is empty, there are no suitable points.
        if(this.size == 0)
            return false;

        // If this is point's squared distance is low enough, it is suitable, so return true.
        if(this.point.distanceToSquared(target) <= minSquaredDistance)
            return true;

        // If it's possible for the left subtree to contain suitable points, search it, and if a suitable point is found, return true.
        if(this.left.size > 0 && !(this.comparison(target) > Math.sqrt(minSquaredDistance)))
            if(this.left.searchExistence(target, minSquaredDistance))
                return true;

        // If it's possible for the right subtree to contain suitable points, search it, and if a suitable point is found, return true.
        if(this.right.size > 0 && !(this.comparison(target) < -1*Math.sqrt(minSquaredDistance)))
            if(this.right.searchExistence(target, minSquaredDistance))
                return true;

        // There are no suitable points.
        return false;
    }

    // Searches the kdTree for a point that is closest to the target, returning the point and its squaredDistance.
    // Target should be a Vector3 point. 
    // If point is not within minSquaredDistance of the target, return null point and Infinity squaredDistance.
    search(target, minSquaredDistance = Infinity) {
        // Edge case for an empty KDTree.
        if(this.size == 0)
            return [null,Infinity];

        // Get a starting guess by finding where target would belong in the KDTree.
        let node = this;
        while(true) {
            // Look at the subtree that target would belong in.
            let nextNode = (node.belongsLeft(target)? node.left:node.right);
            // If that subtree is empty, the current node is the best guess.
            if(nextNode.size == 0)
                break;
            // Else, go to the subtree to look for better guesses.
            else
                node = nextNode;
        }
        let best = node.point;
        let bestSquaredDistance = target.distanceToSquared(best);

        // If this node is outside minSquaredDistance, it is not suitable, so replace it with null.
        if(bestSquaredDistance > minSquaredDistance) {
            best = null;
            bestSquaredDistance = Infinity;
        }

        // Search the tree using the best guess.
        return this.#searchHelper(target, minSquaredDistance, best, bestSquaredDistance);
    }

    #searchHelper(target, minSquaredDistance, oldBest, oldBestSquaredDistance) {
        // Get the current node's point.
        let contender = this.point;
        let contenderSquaredDistance = target.distanceToSquared(contender);

        let best = oldBest;
        let bestSquaredDistance = oldBestSquaredDistance;
        // If contender is suitable and better than oldBestDistance, switch to it.
        if(contenderSquaredDistance <= minSquaredDistance && contenderSquaredDistance < oldBestSquaredDistance) {
            best = contender;
            bestSquaredDistance = contenderSquaredDistance;
        }
        // If contender is unsuitable or not an improvement, use old best, which may be unsuitable as well, in which case it was already set to null before.

        let cutoffSquaredDistance = Math.min(minSquaredDistance, bestSquaredDistance);
        // If it's possible for the left subtree to contain points that are suitable and better than best, search it.
        if(this.left.size > 0 && !(this.comparison(target) > Math.sqrt(cutoffSquaredDistance)))
            [best, bestSquaredDistance] = this.left.#searchHelper(target, minSquaredDistance, best, bestSquaredDistance);
        // If it's possible for the right subtree to contain better points, search it.
        if(this.right.size > 0 && !(this.comparison(target) < -1*Math.sqrt(cutoffSquaredDistance)))
            [best, bestSquaredDistance] = this.right.#searchHelper(target, minSquaredDistance, best, bestSquaredDistance);

        return [best, bestSquaredDistance];
    }

    // Make an array containing only points in the box created by the min and max Vector3 points.
    subsection(min, max) {
        let insidePoints = [];
        
        // If current node's point is in the box, push it into the array.
        if(min.x <= this.point.x && this.point.x <= max.x
        && min.y <= this.point.y && this.point.y <= max.y
        && min.z <= this.point.z && this.point.z <= max.z
        )
            insidePoints.push(this.point);

        // If it's possible for the left subtree to contain points in the box, search it.
        if(this.left.size > 0 && this.belongsLeft(min))
            insidePoints = insidePoints.concat(this.left.subsection(min, max));
        // If it's possible for the right subtree to contain points in the box, search it.
        if(this.right.size > 0 && !this.belongsLeft(max))
            insidePoints = insidePoints.concat(this.right.subsection(min, max));
        
        return insidePoints;
    }
}