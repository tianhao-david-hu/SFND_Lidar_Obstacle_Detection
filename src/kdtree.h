/* \author Aaron Brown */
// Quiz on implementing kd tree
#pragma once

#include <cmath>


// Structure to represent node of kd tree
template <class PointT> 
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template <typename PointT> 
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	bool ifNear(PointT pt1, PointT pt2, double tolerance)
	{
		double dx = pt1.x - pt2.x;
		double dy = pt1.y - pt2.y;
        double dz = pt1.z - pt2.z;
		bool result = sqrt(dx*dx + dy*dy + dz*dz) <= tolerance;
		return result;
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	void insertHelper(Node<PointT>** node, uint depth, PointT point, int id)
	{
		if(*node==NULL)
		{
			*node = new Node<PointT>(point,id); 
		}
		else
		{
			uint dim_idx = depth%3;
            bool compare_result;
            switch(dim_idx)
            {
                case 0: compare_result = point.x < (*node)->point.x;break;
                case 1: compare_result = point.y < (*node)->point.y;break;
                case 2: compare_result = point.z < (*node)->point.z;break;
            };

			if(compare_result)
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);

		}
	}

	void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		//std::cout<<"Into searchHelper()\n";
		if(node!=NULL)
		{
			//std::cout<<"Node is not null.\n";
			uint dim_idx = depth%3;
            float max,min,node_val,target_val;
            switch(dim_idx)
            {
                case 0: max = target.x+distanceTol;min = target.x-distanceTol;node_val=node->point.x;target_val=target.x;break;
                case 1: max = target.y+distanceTol;min = target.y-distanceTol;node_val=node->point.y;target_val=target.y;break;
                case 2: max = target.z+distanceTol;min = target.z-distanceTol;node_val=node->point.z;target_val=target.z;break;
            };
			
			if(node_val<=max && node_val>=min)
			{//need to search both children
				//std::cout<<"Division point in range.\n";
				if(ifNear(target, node->point, distanceTol))
					ids.push_back(node->id);
				searchHelper(target, node->left,  depth+1, distanceTol, ids);
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
			else if(target_val<node_val)
			{//search left
				//std::cout<<"Search left leaf.\n";
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			else if(target_val>node_val)
			{//search right
				//std::cout<<"Search right leaf.\n";
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
		//else
		//	std::cout<<"End of the branch is reached.\n";
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		//std::cout<<"Searching X:"<<target[0]<<" Y:"<<target[1]<<"\n";
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




