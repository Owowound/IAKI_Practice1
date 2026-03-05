// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder.h"

#include <queue>

#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"


APathfinder::APathfinder()
{
	PrimaryActorTick.bCanEverTick = false;
    
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    
	Bounds = CreateDefaultSubobject<UBoxComponent>(TEXT("Bounds"));
	Bounds->SetupAttachment(RootComponent);
	Bounds->SetBoxExtent(FVector(10, 10, 10));
	Bounds->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Bounds->SetHiddenInGame(false);
	
			
	CollisionChecker = CreateDefaultSubobject<UBoxComponent>(TEXT("CollisionParams"));
	CollisionChecker->SetupAttachment(RootComponent);
	CollisionChecker->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	CollisionChecker->SetCollisionResponseToAllChannels(ECR_Overlap);
	CollisionChecker->SetHiddenInGame(true);
	CollisionChecker->SetVisibility(false);
}

void APathfinder::BeginPlay()
{
	Super::BeginPlay();

	// Получение данных о поле
	GridLocation = Bounds->GetComponentLocation();
	GridSize = Bounds->GetScaledBoxExtent();
	
	Height = static_cast<int>(GridSize.X) / static_cast<int>(CellSize) * 2;
	Width  = static_cast<int>(GridSize.Y) / static_cast<int>(CellSize) * 2;
	
	TopLeft = GridLocation + FVector(-static_cast<float>(Height) * CellSize / 2, -static_cast<float>(Width) * CellSize / 2, 0);
	
	cells = std::move(std::vector<std::vector<CellData>>(Height, std::vector<CellData>(Width)));
	
	// Получение данных о входе/выходе
	StartIndx = FindCell(Start);
	EndIndx = FindCell(End);
	
	// Настройка коллищий
	CollisionParams = FCollisionQueryParams::DefaultQueryParam;
	CollisionParams.bTraceComplex = true;
	CollisionParams.AddIgnoredActor(this); 
	
	FVector WantedExtent = FVector(CellSize / 2, CellSize / 2, CellSize / 2);
	FVector ActorScale = GetActorScale3D();
	FVector CorrectedCollisionExtent = FVector(
		WantedExtent.X / ActorScale.X,
		WantedExtent.Y / ActorScale.Y,
		WantedExtent.Z / ActorScale.Z
	);
	CollisionChecker->SetBoxExtent(CorrectedCollisionExtent);
	
	
	// Дебаг старт
	FVector StartLocation = FVector(TopLeft.X + CellSize / 2 + CellSize * StartIndx.first, TopLeft.Y + CellSize / 2 + CellSize * StartIndx.second, GridLocation.Z);
	DrawDebugSphere(
		GetWorld(),
		StartLocation,
		CellSize / 2,
		8,
		FColor::Red,
		false,
		10.0f,
		0,
		2.0f
	);
	
	// Дебаг конец
	FVector EndLocation = FVector(TopLeft.X + CellSize / 2 + CellSize * EndIndx.first, TopLeft.Y + CellSize / 2 + CellSize * EndIndx.second, GridLocation.Z);
	DrawDebugSphere(
		GetWorld(),
		EndLocation,
		CellSize / 2,
		8,
		FColor::Cyan,
		false,
		10.0f,
		0,
		2.0f
		);
	
	CreateGrid();
}

// Получение координат через индексы
FVector APathfinder::getCordsByIndx(int I, int J)
{
	return FVector(TopLeft.X + CellSize / 2 + CellSize * I, TopLeft.Y + CellSize / 2 + CellSize * J, GridLocation.Z);
}


// Построение ячеек поля
void APathfinder::CreateGrid()
{		
		for (int i = 0; i < Height; i++)
		{
			for (int j = 0; j < Width; j++)
			{
				CreateCell();
			}
		}
		
	if (isDijkstra)
	{
		FindPathDijkstra();
	}
	else
	{
		FindPathAstar();
	}
}

// СОздание отдельной ячейки и запись в двумерный вектор
void APathfinder::CreateCell()
{
	FVector CellLocation = getCordsByIndx(currI, currJ);
	FVector CellRelativeLocation = TopLeft - CellLocation;
			
	
	if (currI == StartIndx.first && currJ == StartIndx.second || currI == EndIndx.first && currJ == EndIndx.second)
	{
		if (currI == EndIndx.first && currJ == EndIndx.second)
		{
			cells[currI][currJ] = CellData(CellLocation, CellRelativeLocation, true, true);
		} 
		else
		{
			cells[currI][currJ] = CellData(CellLocation, CellRelativeLocation, true);
		}
	}
	else
	{
		if (!IsItBlocked(currI, currJ))
		{
			cells[currI][currJ] = CellData(CellLocation, CellRelativeLocation, true);
		}
		else
		{
			cells[currI][currJ] = CellData(CellLocation, CellRelativeLocation);
		}
	}
	
	currJ++;
	if (currJ >= Width)
	{
		currI++;
		currJ = 0;
	}
}

// Визуализация точки
void APathfinder::VisualizeCell()
{
	std::pair<int, int> CellID = DijkstraQueue.front();
	DijkstraQueue.pop();
	
	CellData Cell = cells[CellID.first][CellID.second];
	
	DrawDebugSphere(
		GetWorld(),
		Cell.Location,
		CellSize / 2,
		2,
		GridColor,
		false,
		100.0f,
		0,
		2.0f
	);
	
	// Запуск визуализации пути
	if (DijkstraQueue.empty())
	{
		GetWorldTimerManager().ClearTimer(ProcessTimerHandle);
		
		pathIterator = 0;
		GetWorldTimerManager().SetTimer(
			PathTimerHandle,
			this,
			&APathfinder::VisualizeCellPath,
			0.05f,
			true
		);
		return;
	}
}

// Визуализация точки в пути
void APathfinder::VisualizeCellPath()
{	
	DrawDebugSolidBox(
			GetWorld(),
			getCordsByIndx(path[pathIterator].first, path[pathIterator].second),
			FVector(CellSize / 2, CellSize / 2, CellSize / 2),
			PathColor,
			false,
			100.0f,
			0
			);
	
	pathIterator++;
	if (pathIterator >= path.size())
	{
		GetWorldTimerManager().ClearTimer(PathTimerHandle);
	}
}

// Поиск ячеки, которой принадлежит актор
std::pair<int, int> APathfinder::FindCell(AActor* actor)
{
	std::pair<int, int> Cell;
	
	FVector ActorLocation = actor->GetActorLocation();
	
	FVector RelativeLocation = ActorLocation - TopLeft;
	
	int i = static_cast<int>(RelativeLocation.X) / static_cast<int>(CellSize);
	int j = static_cast<int>(RelativeLocation.Y) / static_cast<int>(CellSize);
    
	i = std::max(0, std::min(i, Height - 1));
	j = std::max(0, std::min(j, Width - 1)); 
	
	return {i, j};
}

// Проверка на заблокированность ячейки
bool APathfinder::IsItBlocked(int i, int j)
{
	FVector location = getCordsByIndx(i, j);
	CollisionChecker->SetWorldLocation(location);
	
		TArray<FOverlapResult> Overlaps;
	
		GetWorld()->OverlapMultiByChannel(
			Overlaps,
			location,      
			FQuat::Identity,
			ECC_WorldStatic,
			FCollisionShape::MakeBox(CollisionChecker->GetScaledBoxExtent()),
			CollisionParams
		);
	
		for (const auto& Overlap : Overlaps)
		{
			AActor* actor = Overlap.GetActor();
			if (actor)
			{
				for (FName Tag : actor->Tags)
				{
					if (Tag == "Wall")
					{
						return true;
					}
				}
			}
		}
	
	return false;
}

// Алгоритм Дейкстры
void APathfinder::FindPathDijkstra()
{
	path.clear();
	std::queue<std::pair<int, int>> Queue;
	
	Queue.push(StartIndx);
	DijkstraQueue.push(StartIndx);
	
	std::vector<std::vector<bool>> isVisited(Height, std::vector<bool>(Width, false));
	cells[StartIndx.first][StartIndx.second].distance = 0;
	isVisited[StartIndx.first][StartIndx.second] = true;
	
	while (!Queue.empty())
	{
		std::pair<int, int> cell = Queue.front();
		Queue.pop();
		
		int i = cell.first;
		int j = cell.second;
		
		std::vector<std::pair<int, int>> neighbours = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
		
		for (const auto neighbour : neighbours)
		{
			int I = neighbour.first;
			int J = neighbour.second;
			
			if (I < 0 || I >= Height || J < 0 || J >= Width
				|| !cells[I][J].isActive
				|| isVisited[I][J])
			{
				continue;
			}
			
			float NewDistance = cells[i][j].distance + 1;
			
			if (NewDistance < cells[I][J].distance)
			{
				cells[I][J].distance = NewDistance;
				Queue.push({I, J});
				cells[I][J].parent = {i, j};
				Queue.push({I, J});
				DijkstraQueue.push({I, J});
			}
		}
	}
	
	// Восстановление пути
	int i = EndIndx.first;
	int j = EndIndx.second;
	
	while (i != -1 && j != -1)
	{
		path.push_back({i, j});
		
		int next_i = cells[i][j].parent.first;
		int next_j = cells[i][j].parent.second;

		i = next_i;
		j = next_j;
	}
	
	GetWorldTimerManager().SetTimer(
		ProcessTimerHandle,
		this,
		&APathfinder::VisualizeCell,
		0.005f,
		true
	);
}

// Расстояние от точки 1 до 2
int APathfinder::Heuristic(int i1, int j1, int i2, int j2)
{
	return std::abs(i1 - i2) + abs(j1 - j2);
}

// Алгоритм A*
void APathfinder::FindPathAstar()
{
	path.clear();
	
	auto compare = [](const std::pair<float, std::pair<int, int>>& a, const std::pair<float, std::pair<int, int>>& b)
	{
		return a.first > b.first;
	};
	std::priority_queue<std::pair<float, std::pair<int, int>>, std::vector<std::pair<float, std::pair<int, int>>>, decltype(compare)> Queue(compare);
	
	cells[StartIndx.first][StartIndx.second].heuristic = Heuristic(StartIndx.first, StartIndx.second, EndIndx.first, EndIndx.second);
	cells[StartIndx.first][StartIndx.second].distance = 0;
	float FCost = cells[StartIndx.first][StartIndx.second].heuristic + cells[StartIndx.first][StartIndx.second].distance;
	
	Queue.push({FCost, StartIndx});
	DijkstraQueue.push(StartIndx);
	
	std::vector<std::vector<bool>> isVisited(Height, std::vector<bool>(Width, false));
	isVisited[StartIndx.first][StartIndx.second] = true;
	
	while (!Queue.empty())
	{
		std::pair<int, int> cell = Queue.top().second;
		Queue.pop();
		isVisited[cell.first][cell.second] = true;
		
		int i = cell.first;
		int j = cell.second;
		if (i == EndIndx.first && j == EndIndx.second)
		{
			break;
		}
		
		std::vector<std::pair<int, int>> neighbours = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
		
		for (const auto neighbour : neighbours)
		{
			int I = neighbour.first;
			int J = neighbour.second;
			
			if (I < 0 || I >= Height || J < 0 || J >= Width
				|| !cells[I][J].isActive
				|| isVisited[I][J])
			{
				continue;
			}
			
			float NewDistance = cells[i][j].distance + 1;
			
			if (NewDistance < cells[I][J].distance)
			{
				cells[I][J].parent = {i, j};
				cells[I][J].distance = NewDistance;
				cells[I][J].heuristic = Heuristic(I, J, EndIndx.first, EndIndx.second);
				
				float NewFCost = cells[I][J].heuristic + cells[I][J].distance;
				Queue.push({NewFCost, {I, J}});
				DijkstraQueue.push({I, J});
			}
		}
	}
	
	// Восстановдение пути
	int i = EndIndx.first;
	int j = EndIndx.second;
	
	while (i != -1 && j != -1)
	{
		path.push_back({i, j});
		
		int next_i = cells[i][j].parent.first;
		int next_j = cells[i][j].parent.second;

		i = next_i;
		j = next_j;
	}
	
	UE_LOG(LogTemp, Warning, TEXT("=== PATH DEBUG ==="));
	UE_LOG(LogTemp, Warning, TEXT("Path size: %d"), path.size());

	for (int idx = 0; idx < path.size(); idx++)
	{
		UE_LOG(LogTemp, Log, TEXT("path[%d] = [%d, %d]"), 
			idx, path[idx].first, path[idx].second);
	}
	
	GetWorldTimerManager().SetTimer(
		ProcessTimerHandle,
		this,
		&APathfinder::VisualizeCell,
		0.005f,
		true
	);
}