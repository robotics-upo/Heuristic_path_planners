#include "Grid3D/gridMap.hpp"

namespace Planners
{
    namespace utils
    {
        GridMap::GridMap()
        {
        }
        bool GridMap::readOctomap(const std::string &_path)
        {

            if (!std::filesystem::exists(_path))
            {
                std::cout << "File " << _path << " does not exist! Aborting " << std::endl;
                return false;
            }
            if (_path.substr(_path.length() - 3, 3) != ".bt")
            {
                std::cout << "File " << _path << " does not have .bt extension! Aborting " << std::endl;
                return false;
            }

            octomap::AbstractOcTree *tree;
            octomap::OcTree *binaryTree = new octomap::OcTree(octree_resolution_);

            if (binaryTree->readBinary(_path) && binaryTree->size() > 1)
            {
                tree = binaryTree;
            }
            else
            {
                return false;
            }
            
            return true;
        }
    } // namespace utis

} // namespace Planners
