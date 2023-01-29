/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	bool ifNear(std::vector<float> &pt1, std::vector<float> &pt2, double tolerance)
	{
		double dx = pt1[0] - pt2[0];
		double dy = pt1[1] - pt2[1];
		bool result = sqrt(dx*dx + dy*dy) <= tolerance;
		if(result)
			std::cout<<"Find neighbour point X:"<<pt2[0] << "Y:"<<pt2[1]<<"\n";
		return result;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	void insertHelper(Node** node, uint depth, std::vector<float>point, int id)
	{
		if(*node==NULL)
		{
			*node = new Node(point,id); 
		}
		else
		{
			uint dimention_index = depth%2;
			if(point[dimention_index] < (*node)->point[dimention_index])
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);

		}
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		//std::cout<<"Into searchHelper()\n";
		if(node!=NULL)
		{
			//std::cout<<"Node is not null.\n";
			uint dim_idx = depth%2;
			float max = target[dim_idx]+distanceTol;
			float min = target[dim_idx]-distanceTol;
			if(node->point[dim_idx]<=max && node->point[dim_idx]>=min)
			{//need to search both children
				//std::cout<<"Division point in range.\n";
				if(ifNear(target, node->point, distanceTol))
					ids.push_back(node->id);
				searchHelper(target, node->left,  depth+1, distanceTol, ids);
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
			else if(target[dim_idx]<node->point[dim_idx])
			{//search left
				//std::cout<<"Search left leaf.\n";
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			else if(target[dim_idx]>node->point[dim_idx])
			{//search right
				//std::cout<<"Search right leaf.\n";
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
		//else
		//	std::cout<<"End of the branch is reached.\n";
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::cout<<"Searching X:"<<target[0]<<" Y:"<<target[1]<<"\n";
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




