// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <queue>
#include <vector>

#include "CoreMinimal.h"
#include "CellData.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "Pathfinder.generated.h"

UCLASS(BLueprintable)
class DIJKSTRA_API APathfinder : public AActor
{
	GENERATED_BODY()

public:
	APathfinder();
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	UBoxComponent* Bounds;
	
	UPROPERTY(EditAnywhere)
	float CellSize = 20.0f;

	UPROPERTY(EditAnywhere)
	FColor GridColor;
	
	UPROPERTY(EditAnywhere)
	FColor PathColor;
	
	UPROPERTY(EditAnywhere)
	AActor* Start;
	
	UPROPERTY(EditAnywhere)
	AActor* End;
	
	UPROPERTY(EditAnywhere)
	bool isDijkstra = true;
	
private:
	std::vector<std::vector<CellData>> cells;
	std::vector<std::pair<int, int>> path;
	std::queue<std::pair<int, int>> DijkstraQueue;
	
	int Height;
	int Width;
	FVector TopLeft;
	
	int currI = 0;
	int currJ = 0;
	
	int pathIterator = 0;
	int gridIterator = 0;
	
	FVector GridLocation;
	FVector GridSize;
	
	std::pair<int, int> StartIndx;
	std::pair<int, int> EndIndx;
	
	UBoxComponent* CollisionChecker;
	FCollisionQueryParams CollisionParams;
	
	FTimerHandle GridTimerHandle;
	FTimerHandle PathTimerHandle;
	FTimerHandle FindPathDelayHandle; 
	FTimerHandle ProcessTimerHandle; 
	
protected:
	virtual void BeginPlay() override;

	FVector getCordsByIndx(int I, int J);
	
	void CreateGrid();
	
	void CreateCell();
	
	void VisualizeCell();
	void VisualizeCellPath();
	
	std::pair<int, int> FindCell(AActor* actor);
	
	bool IsItBlocked(int i, int j);
	
	void FindPathDijkstra();
	void FindPathAstar();
	
	int Heuristic(int i1, int j1, int i2, int j2);
};
