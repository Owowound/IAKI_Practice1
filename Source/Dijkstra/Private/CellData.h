#pragma once


struct CellData
{
	FVector Location;
	FVector RelativeLocation;
	bool isActive = false;
	bool isEnd = false;
	bool isPath = false;
	int distance = INT_MAX;
	
	int heuristic = 0;
	
	std::pair<int, int> parent {-1, -1};
};
