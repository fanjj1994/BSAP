#ifndef LOCAL_PATH_CREATOR_H_
#define LOCAL_PATH_CREATOR_H_

#include "local_path_solver_interface.h"

namespace Planning
{
    class LocalPathCreator
    {
    public:
        LocalPathCreator();

        // 参考线 + 决策点 -> 局部路径
        base_msgs::msg::LocalPath createLocalPath(const base_msgs::msg::Referline& referenceLine, const std::vector<SLPoint>& decisionPoints);
        // 局部路径转 RViz 显示
        nav_msgs::msg::Path localPathToRviz();
        inline base_msgs::msg::LocalPath getLocalPath() const { return localPath; }
    private:
        std::unique_ptr<ConfigReader> localPathConfigReader;
        std::shared_ptr<LocalPathSolverInterface> localPathSolver;
        base_msgs::msg::LocalPath localPath;
        nav_msgs::msg::Path localPathRviz;
    };

}  // namespace Planning
#endif  // LOCAL_PATH_CREATOR_H_
