#include "planner/planner.hpp"

Planner::Planner()
{
    printf("class test has been constructed");
}

Planner::~Planner()
{
    printf("class test has been destructed");
}

void Planner::print(std::string str)
{
    std::cout<<str<<"\n";
}
