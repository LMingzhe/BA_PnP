#include "BFMatch.h"
#include "TypeDef.h"
#include "Match.h"

std::vector<Match> bfMatch(std::vector<DesType> &desc1, std::vector<DesType> &desc2)
{
    std::cout << "====================start bfMatch...====================" << std::endl;

    std::vector<Match> matches_;
    const int d_max = 50;
    for (int i = 0; i < desc1.size(); i++) 
    {
        DesType d1 = desc1[i];
        // 判断是否为空
        if (d1.empty())
        {
            continue;
        }
        // 寻找最短的匹配点
        int minDist = d_max + 1;
        int minIndex;
        for (int j = 0; j < desc2.size(); j++)
        {
            DesType d2 = desc2[j];
            int theDist = 0;
            if (d2.empty())
            {
                continue;
            }
            for (int k = 0; k < d1.size() && k < d2.size(); k++)
            {
                if (d1[k] != d2[k])
                {
                    theDist++;
                }
            }
            if (theDist < minDist)
            {
                minDist = theDist;
                minIndex = j;
            }
        }
        if (minDist <= d_max)
        {
            Match match_(i, minIndex, minDist);
            matches_.push_back(match_);
        }
    }

    // 输出匹配点信息
    for (auto match : matches_)
    {
        std::cout << "id1:" << match.getID1() << ", id2:" << match.getID2() <<", distance:" << match.getDistance() << std::endl; 
    }

    std::cout << "====================bfMatch is over...====================" << std::endl;
    
    return matches_;
}