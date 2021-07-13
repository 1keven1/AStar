#include <iostream>
#include <Windows.h>
#include <ctime>
#include <fstream>
#include <opencv2/opencv.hpp>
/*
* Created by Suzhi_Zhang on 2021/5/26
* https://1keven1.github.io/
*/
/*
* ���߳�A*�㷨ʵ��
* ����ʹ����OpenCV�⣬����û������OpenCV���޷�����
* VS2019 OpenCV���÷�ʽ��https://1keven1.github.io/2021/01/29/%E3%80%90C-%E3%80%91VS2019%E9%85%8D%E7%BD%AEOpenCV%E5%B9%B6%E8%BF%9B%E8%A1%8C%E6%B5%8B%E8%AF%95/
* һ�����������Ͻ�Ϊԭ�㣬��0��ʼ����������X������Y��
*/

// ȫ�ֱ���
int windowHeight;		// ��Ļ�߶�
int windowWidth;		// ��Ļ���
int gridNumX;			// ��������
int gridNumY;			// ��������
int gridLineWidth;		// ������߿��

// ����
const cv::Vec3f backgroundColor = cv::Vec3f(0.2, 0.2, 0.2);				// ������ɫ
const cv::Vec3f gridLineColor = cv::Vec3f(1, 1, 1);						// ���������ɫ
const cv::Vec3f normalGridBackgroundColor = cv::Vec3f(0.1, 0.1, 0.1);	// ��ͨ������ɫ
const cv::Vec3f blockColor = cv::Vec3f(0.95, 0.95, 0.95);				// ����ͨ��������ɫ
const cv::Vec3f startColor = cv::Vec3f(0, 1, 0);						// ��ʼ������ɫ
const cv::Vec3f goalColor = cv::Vec3f(1, 0, 1);							// Ŀ��������ɫ
const cv::Vec3f routeColor = cv::Vec3f(0.7, 0.5, 0);					// ·��������ɫ

// ͨ����Ļ�����ȡ���ص������±�
int GetFrameBufferNumByCoord(int x, int y)
{
	return y * windowWidth + x;
}

// ��������
enum GridType
{
	NORMAL,
	BLOCK,
	START,
	GOAL,
	ROUTE
};

// ����ṹ��
struct Grid
{
public:
	// ���캯��
	Grid()
	{
		_x = 0;
		_y = 0;
		_centerX = 0;
		_centerY = 0;
		_height = 0;
		_width = 0;
		bBlock = false;
		_type = NORMAL;
	}
	Grid(const int x, const int y)
	{
		_x = x;
		_y = y;
		_centerX = 0;
		_centerY = 0;
		_height = 0;
		_width = 0;
		bBlock = false;
		_type = NORMAL;
	}

	// ����������
	void SetIndex(const int& x, const int& y)
	{
		_x = x;
		_y = y;
	}

	// ���������ͨ����
	void SetBlock(const bool& b)
	{
		bBlock = b;
		if (b) _type = BLOCK;
		else _type = NORMAL;
	}

	// ������������
	void SetData(const float& x, const float& y, const float& width, const float& height)
	{
		_centerX = x;
		_centerY = y;
		_width = width;
		_height = height;
	}

	// ������������
	void SetType(const GridType& type)
	{
		_type = type;
	}

	// ������������
	void SetParent(Grid* const parent)
	{
		_parent = parent;
	}

	// TODO: ������cost���ڽṹ����
	// ��������Cost
	void SetCost4Way(Grid* const goal)
	{
		// GCost
		if (_type == START) // ����ǿ�ʼ����
		{
			_gCost = 0;
		}
		else // ������ǿ�ʼ����
		{
			_gCost = _parent->GetGCost() + 1;
		}

		// HCost
		int endX, endY;
		std::tie(endX, endY) = goal->GetIndex();
		_hCost = abs(endX - _x) + abs(endY - _y);

		// FCost
		_fCost = _gCost + _hCost;
	}
	void SetCost8Way(Grid* const goal)
	{
		// GCost
		if (_type == START) // ����ǿ�ʼ����
		{
			_gCost = 0;
		}
		else // ������ǿ�ʼ����
		{
			int parentX, parentY;
			std::tie(parentX, parentY) = _parent->GetIndex();
			if (parentX == _x || parentY == _y)
			{
				_gCost = _parent->GetGCost() + 10;
			}
			else
			{
				_gCost = _parent->GetGCost() + 14;
			}
		}

		// HCost
		int endX, endY;
		std::tie(endX, endY) = goal->GetIndex();
		int dx = abs(endX - _x);
		int dy = abs(endY - _y);
		_hCost = std::min(dx, dy) * 14 + abs(dx - dy) * 10;

		// FCost
		_fCost = _gCost + _hCost;
	}

	// ��ȡ�����ͨ����
	const bool GetBlock()const
	{
		return bBlock;
	}

	// ��ȡ��������
	const GridType GetType()const
	{
		return _type;
	}

	// ��ȡ����GCost
	const int GetGCost()const
	{
		return _gCost;
	}

	// ��ȡ����HCost
	const int GetHCost()const
	{
		return _hCost;
	}

	// ��ȡ����FCost
	const int GetFCost()const
	{
		return _fCost;
	}

	// ��ȡ��������
	Grid* const GetParent()const
	{
		return _parent;
	}

	// ��ȡ������
	const std::tuple<float, float> GetIndex()const
	{
		return { _x, _y };
	}

	// ��������
	void Draw(std::vector<cv::Vec3f>& frameBuffer)const
	{
		for (int j = (int)(_centerY - _height * 0.5); j < (int)(_centerY + _height * 0.5); j++)
		{
			for (int i = (int)(_centerX - _width * 0.5); i < (int)(_centerX + _width * 0.5); i++)
			{
				// ��ֹ����Խ��
				if (j > windowHeight - 1) i = windowHeight - 1;
				if (i > windowWidth - 1) j = windowWidth - 1;
				// ����
				switch (_type)
				{
				case NORMAL:
					if (abs(i - _centerX) > _width * 0.5 - gridLineWidth || abs(j - _centerY) > _height * 0.5 - gridLineWidth)
						frameBuffer[GetFrameBufferNumByCoord(i, j)] = gridLineColor;
					else frameBuffer[GetFrameBufferNumByCoord(i, j)] = normalGridBackgroundColor;
					break;
				case BLOCK:
					if (abs(i - _centerX) > _width * 0.5 - gridLineWidth || abs(j - _centerY) > _height * 0.5 - gridLineWidth)
						frameBuffer[GetFrameBufferNumByCoord(i, j)] = gridLineColor;
					else frameBuffer[GetFrameBufferNumByCoord(i, j)] = blockColor;
					break;
				case START:
					if (abs(i - _centerX) > _width * 0.5 - gridLineWidth || abs(j - _centerY) > _height * 0.5 - gridLineWidth)
						frameBuffer[GetFrameBufferNumByCoord(i, j)] = gridLineColor;
					else frameBuffer[GetFrameBufferNumByCoord(i, j)] = startColor;
					break;
				case GOAL:
					if (abs(i - _centerX) > _width * 0.5 - gridLineWidth || abs(j - _centerY) > _height * 0.5 - gridLineWidth)
						frameBuffer[GetFrameBufferNumByCoord(i, j)] = gridLineColor;
					else frameBuffer[GetFrameBufferNumByCoord(i, j)] = goalColor;
					break;
				case ROUTE:
					if (abs(i - _centerX) > _width * 0.5 - gridLineWidth || abs(j - _centerY) > _height * 0.5 - gridLineWidth)
						frameBuffer[GetFrameBufferNumByCoord(i, j)] = gridLineColor;
					else frameBuffer[GetFrameBufferNumByCoord(i, j)] = routeColor;
				}
			}
		}
	}

private:
	int _x, _y;					// ������
	float _centerX, _centerY;	// �������ĵ���Ļ����
	float _width, _height;		// ���񳤿�
	GridType _type;				// ��������
	bool bBlock;				// �����ͨ����

	// A*�㷨���ñ���
	int _gCost = 0;				// ��ʼ���߹����Ĵ���
	int _hCost = 0;				// ����Ŀ���ľ���
	int _fCost = 0;
	Grid* _parent = NULL;		// ������
};

// ��ʾ�������
std::vector<Grid> grids;			// ��������
std::vector<cv::Vec3f> frameBuffer;	// Frame Buffer
Grid* startGrid = NULL;				// ��ʼ��
Grid* goalGrid = NULL;				// ������
std::vector<Grid*> route;			// ·����
std::function<bool()> aStarFunc;	// ����A���㷨�����ĺ���ָ��

// ͨ�������Ż�ȡ���������±�
int GetGridNumByIndex(const int& x, const int& y)
{
	return y * gridNumX + x;
}

// ͨ����Ļ�����ȡ������
std::tuple<int, int> GetGridIndexByCoord(const int& x, const int& y)
{
	float gridWidth = static_cast<float>(windowWidth) / gridNumX;
	float gridHeight = static_cast<float>(windowHeight) / gridNumY;
	int _x, _y;
	_x = floor(x / gridWidth);
	_y = floor(y / gridHeight);
	return { _x, _y };
}

// ͨ����Ļ�������������ͨ����
void SetGridBlockByCoord(const int& x, const int& y, const bool& bBlock)
{
	int gridNumX, gridNumY;
	std::tie(gridNumX, gridNumY) = GetGridIndexByCoord(x, y);
	grids[GetGridNumByIndex(gridNumX, gridNumY)].SetBlock(bBlock);
}

// ͨ�����������������ͨ����
void SetGridBlockByIndex(const int& x, const int& y, const bool& bBlock)
{
	grids[GetGridNumByIndex(x, y)].SetBlock(bBlock);
}

// ͨ����Ļ�����л������ͨ����
void ToggleGridBlockByCoord(const int& x, const int& y)
{
	int gridNumX, gridNumY;
	std::tie(gridNumX, gridNumY) = GetGridIndexByCoord(x, y);
	if (grids[GetGridNumByIndex(gridNumX, gridNumY)].GetBlock())
	{
		grids[GetGridNumByIndex(gridNumX, gridNumY)].SetBlock(false);
	}
	else
	{
		grids[GetGridNumByIndex(gridNumX, gridNumY)].SetBlock(true);
	}
}

// ͨ����Ļ����������������
void SetGridTypeByCoord(const int& x, const int& y, const GridType& type)
{
	int gridNumX, gridNumY;
	std::tie(gridNumX, gridNumY) = GetGridIndexByCoord(x, y);
	grids[GetGridNumByIndex(gridNumX, gridNumY)].SetType(type);
	std::string tp;
	switch (type)
	{
	case NORMAL:
		tp = "��������";
		break;
	case START:
		tp = "��ʼ��";
		break;
	case GOAL:
		tp = "Ŀ���";
		break;
	default:
		break;
	}
	std::cout << "������" << gridNumX << ", " << gridNumY << "�����ó�" << tp << std::endl;
}

// OpenCV: ����¼����
bool bHoldRButton = false;
GridType type;
int mouseX = -1;
int mouseY = -1;
void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
	// �������ڴ��������ʼ��
	if (x < 0 || y < 0 || x > windowWidth || y > windowHeight)
	{
		bHoldRButton = false;
		mouseX = -1;
		mouseY = -1;
		return;
	}

	// ����������
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		int gridIndexX, gridIndexY;
		std::tie(gridIndexX, gridIndexY) = GetGridIndexByCoord(x, y);
		int gridNum = GetGridNumByIndex(gridIndexX, gridIndexY);
		// ���ڲ�ͬ���������
		switch (grids[gridNum].GetType())
		{
		case NORMAL:
			if (!startGrid)
			{
				SetGridTypeByCoord(x, y, START);
				startGrid = &grids[gridNum];
				return;
			}
			if (!goalGrid)
			{
				SetGridTypeByCoord(x, y, GOAL);
				goalGrid = &grids[gridNum];
				return;
			}
			return;
			break;
		case BLOCK:
			if (!startGrid)
			{
				SetGridTypeByCoord(x, y, START);
				startGrid = &grids[gridNum];
				return;
			}
			if (!goalGrid)
			{
				SetGridTypeByCoord(x, y, GOAL);
				goalGrid = &grids[gridNum];
				return;
			}
			return;
			break;
		case START:
			SetGridTypeByCoord(x, y, NORMAL);
			startGrid = NULL;
			return;
			break;
		case GOAL:
			SetGridTypeByCoord(x, y, NORMAL);
			goalGrid = NULL;
			return;
			break;
		}
	}

	// ��������Ҽ�
	if (event == cv::EVENT_RBUTTONDOWN)
	{
		bHoldRButton = true;
		mouseX = x;
		mouseY = y;
		int gridIndexX, gridIndexY;
		std::tie(gridIndexX, gridIndexY) = GetGridIndexByCoord(x, y);
		int gridNum = GetGridNumByIndex(gridIndexX, gridIndexY);
		type = grids[gridNum].GetType();
	}

	// �ɿ�����Ҽ�
	if (event == cv::EVENT_RBUTTONUP)
	{
		bHoldRButton = false;
		if (mouseX == x && mouseY == y)
		{
			int gridIndexX, gridIndexY;
			std::tie(gridIndexX, gridIndexY) = GetGridIndexByCoord(x, y);
			int gridNum = GetGridNumByIndex(gridIndexX, gridIndexY);
			switch (grids[gridNum].GetType())
			{
			case NORMAL:
				SetGridBlockByCoord(x, y, true);
				return;
				break;
			case BLOCK:
				SetGridBlockByCoord(x, y, false);
				return;
				break;
			case START:
				SetGridBlockByCoord(x, y, true);
				startGrid = NULL;
				return;
				break;
			case GOAL:
				SetGridBlockByCoord(x, y, true);
				goalGrid = NULL;
				return;
				break;
			}
			mouseX = -1;
			mouseY = -1;
		}
	}

	// ����ƶ�
	if (event == cv::EVENT_MOUSEMOVE)
	{
		if (bHoldRButton)
		{
			int gridIndexX, gridIndexY;
			std::tie(gridIndexX, gridIndexY) = GetGridIndexByCoord(x, y);
			int gridNum = GetGridNumByIndex(gridIndexX, gridIndexY);
			bool bBlock = type == BLOCK ? false : true;
			// ���ڲ�ͬ���������
			switch (grids[gridNum].GetType())
			{
			case NORMAL:
				SetGridBlockByCoord(x, y, bBlock);
				return;
				break;
			case BLOCK:
				SetGridBlockByCoord(x, y, bBlock);
				return;
				break;
			case START:
				SetGridBlockByCoord(x, y, bBlock);
				startGrid = NULL;
				return;
				break;
			case GOAL:
				SetGridBlockByCoord(x, y, bBlock);
				goalGrid = NULL;
				return;
				break;
			}
		}
	}
}

// ��ʼ��Frame Buffer����������
void Initialization(const int& windowX, const int& windowY, const int& gridX, const int& gridY)
{
	// ��ʼ��Frame Buffer
	frameBuffer.resize(windowX * windowY);
	std::fill(frameBuffer.begin(), frameBuffer.end(), backgroundColor);
	// ��ʼ��Grids����
	grids.resize(gridX * gridY);
	std::fill(grids.begin(), grids.end(), Grid());
	// ����Grid������
	for (int j = 0; j < gridY; j++)
	{
		for (int i = 0; i < gridX; i++)
		{
			float width = static_cast<float>(windowY) / gridX;
			float height = static_cast<float>(windowX) / gridY;
			grids[GetGridNumByIndex(i, j)].SetIndex(i, j);
			grids[GetGridNumByIndex(i, j)].SetData(width * (float)(i + 0.5f), height * (float)(j + 0.5f), width, height);
		}
	}
}

// A*�㷨���������Ƿ����Ԫ��
bool VectorContainItem(std::vector<Grid*> vector, Grid* const item)
{
	for (auto _item : vector)
	{
		if (_item == item) return true;
	}
	return false;
}

// A*�㷨����ȡ�����ھ�
std::vector<Grid*> FindAllNeighbors4Way(Grid* const current)
{
	std::vector<Grid*> neighbors;
	int currentX, currentY;
	std::tie(currentX, currentY) = current->GetIndex();
	// ��
	if (currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// ��
	if (currentY < gridNumY - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY + 1)];
		neighbors.push_back(neighbor);
	}

	// ��
	if (currentX > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY)];
		neighbors.push_back(neighbor);
	}

	// ��
	if (currentX < gridNumX - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX + 1, currentY)];
		neighbors.push_back(neighbor);
	}
	return neighbors;
}
std::vector<Grid*> FindAllNeighbors8Way(Grid* const current)
{
	std::vector<Grid*> neighbors;
	int currentX, currentY;
	std::tie(currentX, currentY) = current->GetIndex();
	// ��
	if (currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// ��
	if (currentY < gridNumY - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY + 1)];
		neighbors.push_back(neighbor);
	}

	// ��
	if (currentX > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY)];
		neighbors.push_back(neighbor);
	}

	// ��
	if (currentX < gridNumX - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX + 1, currentY)];
		neighbors.push_back(neighbor);
	}

	// ����
	if (currentX > 0 && currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// ����
	if (currentX > 0 && currentY < gridNumY - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY + 1)];
		neighbors.push_back(neighbor);
	}

	// ����
	if (currentX < gridNumX - 1 && currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX + 1, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// ����
	if (currentX < gridNumX - 1 && currentY < gridNumY - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX + 1, currentY + 1)];
		neighbors.push_back(neighbor);
	}

	return neighbors;
}

int CalculateFCost(Grid* const grid, Grid* const parent, Grid* const goal)
{
	int gCost, hCost, fCost;

	int gridX, gridY;
	std::tie(gridX, gridY) = grid->GetIndex();
	int parentX, parentY;
	std::tie(parentX, parentY) = parent->GetIndex();

	// GCost
	if (parentX == gridX || parentY == gridY)
	{
		gCost = parent->GetGCost() + 10;
	}
	else
	{
		gCost = parent->GetGCost() + 14;
	}

	// HCost
	int endX, endY;
	std::tie(endX, endY) = goal->GetIndex();
	int dx = abs(endX - gridX);
	int dy = abs(endY - gridY);
	hCost = std::min(dx, dy) * 14 + abs(dx - dy) * 10;

	// FCost
	fCost = gCost + hCost;

	return fCost;
}

// A*�㷨����
bool AStar4Way()
{
	// A*�㷨�������
	std::vector<Grid*> openGrid;	// openList
	std::vector<Grid*> closeGrid;	// closeList
	Grid* current = NULL;			// ���ڼ���Ľڵ�

	std::cout << "��ʼ����ͨѰ·" << std::endl;
	clock_t start = clock();

	// ����ʼ�ڵ����Open List
	startGrid->SetCost4Way(goalGrid);
	openGrid.push_back(startGrid);

	// ��Open List����Ԫ�ص�ʱ�����ѭ��
	while (openGrid.size() > 0)
	{
		// ��current�������ó�Open List��FCost��С�ģ����FCostһ���Ļ����ó�HCost��С��
		int minFCost = std::numeric_limits<int>::max();
		int currentNum = 0;
		int i = 0;
		for (auto grid : openGrid)
		{
			if (grid->GetFCost() < minFCost)
			{
				current = grid;
				minFCost = grid->GetFCost();
				currentNum = i;
			}
			else if (grid->GetFCost() == minFCost && current)
			{
				if (grid->GetHCost() < current->GetHCost())
				{
					current = grid;
					currentNum = i;
				}
			}
			i++;
		}

		// ��current��open list���Ƴ������뵽close list��
		openGrid.erase(openGrid.begin() + currentNum);
		closeGrid.push_back(current);

		// ���current��Ŀ�����Ѱ·�ɹ�
		if (current && current->GetType() == GOAL)
		{
			clock_t end = clock();
			std::cout << "Ѱ·�ɹ�����ʱ��" << end - start << "����" << std::endl;
			return true;
		}

		// ����ÿһ���ھ�����
		auto neighbors = FindAllNeighbors4Way(current);
		for (auto neighbor : neighbors)
		{
			// �����open��close list�л�����ǽ��������
			if (neighbor->GetBlock() || VectorContainItem(closeGrid, neighbor) || VectorContainItem(openGrid, neighbor))
				continue;
			else
			{
				neighbor->SetParent(current);
				neighbor->SetCost4Way(goalGrid);
				openGrid.push_back(neighbor);
			}
		}

	}
	std::cout << "ʧ����" << std::endl;
	return false;
}
bool AStar8Way()
{
	// A*�㷨�������
	std::vector<Grid*> openGrid;	// openList
	std::vector<Grid*> closeGrid;	// closeList
	Grid* current = NULL;			// ���ڼ���Ľڵ�

	std::cout << "��ʼ����ͨѰ·" << std::endl;
	clock_t start = clock();

	// ����ʼ�ڵ����Open List
	startGrid->SetCost8Way(goalGrid);
	openGrid.push_back(startGrid);

	// ��Open List����Ԫ�ص�ʱ�����ѭ��
	while (openGrid.size() > 0)
	{
		// ��current�������ó�Open List��FCost��С�ģ����FCostһ���Ļ����ó�HCost��С��
		int minFCost = std::numeric_limits<int>::max();
		int currentNum = 0;
		int i = 0;
		for (auto grid : openGrid)
		{
			if (grid->GetFCost() < minFCost)
			{
				current = grid;
				minFCost = grid->GetFCost();
				currentNum = i;
			}
			else if (grid->GetFCost() == minFCost && current)
			{
				if (grid->GetHCost() < current->GetHCost())
				{
					current = grid;
					currentNum = i;
				}
			}
			i++;
		}

		// ��current��open list���Ƴ������뵽close list��
		openGrid.erase(openGrid.begin() + currentNum);
		closeGrid.push_back(current);

		// ���current��Ŀ�����Ѱ·�ɹ�
		if (current && current->GetType() == GOAL)
		{
			clock_t end = clock();
			std::cout << "Ѱ·�ɹ�����ʱ��" << end - start << "����" << std::endl;
			return true;
		}

		// ����ÿһ���ھ�����
		auto neighbors = FindAllNeighbors8Way(current);
		for (auto neighbor : neighbors)
		{
			if (neighbor->GetBlock() || VectorContainItem(closeGrid, neighbor))
				continue;

			// �����·��FCost���͵Ļ����¸ýڵ��Cost
			int newFCost = CalculateFCost(neighbor, current, goalGrid);
			if ((newFCost < neighbor->GetFCost()) || (!VectorContainItem(openGrid, neighbor)))
			{
				neighbor->SetParent(current);
				neighbor->SetCost8Way(goalGrid);
				if (!VectorContainItem(openGrid, neighbor))
				{
					openGrid.push_back(neighbor);
				}
			}
		}
	}
	std::cout << "ʧ����" << std::endl;
	return false;
}

// ��������·���ڵ㣬������������
void AddAllRouteGridToVector(Grid* const grid)
{
	if (grid->GetParent())
	{
		route.push_back(grid);
		AddAllRouteGridToVector(grid->GetParent());
	}
	else
	{
		return;
	}
}

// ��ʾ·����
void ShowRoute()
{
	for (auto routeGrid : route)
	{
		if (routeGrid->GetType() != GOAL) routeGrid->SetType(ROUTE);
	}
}

// ��ȡ�����ļ�
void ReadConfig()
{
	bool bReadScreenConfig = false;
	bool bReadGridNum = false;
	bool bReadGridLineWidth = false;
	bool bReadWay = false;
	bool b4Way;

	std::ifstream file("config.txt", std::ios::in);
	if (!file)
	{
		std::cout << "WARNING: ��ȡconfig�ļ�ʧ��" << std::endl;
		system("pause");
		exit(0);
	}

	while (1)
	{
		if (file.eof())
		{
			std::cout << "WARNING: config�ļ��޽���ָ��\"END\"" << std::endl;
			system("pause");
			exit(0);
		}

		std::string s;
		file >> s;
		if (s.compare("4WayOr8Way") == 0)
		{
			int i;
			file >> i;
			if (i == 4)
			{
				aStarFunc = AStar4Way;
				b4Way = true;
			}
			if (i == 8)
			{
				aStarFunc = AStar8Way;
				b4Way = false;
			}
			if (i != 4 && i != 8)
			{
				std::cout << "WARNING: config�����Դ���4WayOr8Way" << std::endl;
				system("pause");
				exit(0);
			}
			bReadWay = true;
		}
		else if (s.compare("ScreenConfig(256-2048)") == 0)
		{
			file >> windowHeight >> windowWidth;
			bReadScreenConfig = true;
		}
		else if (s.compare("GridNum(5-64)") == 0)
		{
			file >> gridNumX >> gridNumY;
			bReadGridNum = true;
		}
		else if (s.compare("GridLineWidth(1-3)") == 0)
		{
			file >> gridLineWidth;
			bReadGridLineWidth = true;
		}
		else if (s.compare("End") == 0)
		{
			bool bSuccess = true;
			if (!bReadScreenConfig)
			{
				std::cout << "WARNING: config����ʧ���ԣ�ScreenConfig" << std::endl;
				bSuccess = false;
			}
			if (!bReadGridNum)
			{
				std::cout << "WARNING: config����ʧ���ԣ�GridNum" << std::endl;
				bSuccess = false;
			}
			if (!bReadGridLineWidth)
			{
				std::cout << "WARNING: config����ʧ���ԣ�GridLineWidth" << std::endl;
				bSuccess = false;
			}
			if (!bReadWay)
			{
				std::cout << "WARNING: config����ʧ���ԣ�4WayOr8Way" << std::endl;
				bSuccess = false;
			}

			if (bSuccess)
			{
				std::cout << "��ȡConfig�ļ��ɹ�" << std::endl;

				if (b4Way) std::cout << "����ͨA*Ѱ·" << std::endl;
				else std::cout << "����ͨA*Ѱ·" << std::endl;
				std::cout << "��Ļ��ߣ�" << windowWidth << "x" << windowHeight << std::endl;
				std::cout << "����������" << gridNumX << "x" << gridNumY << std::endl;
				std::cout << "�����ߴ�ϸ��" << gridLineWidth << std::endl;
				std::cout << "======================================" << std::endl;
				break;
			}
			else
			{
				system("pause");
				exit(0);
			}
		}
		else
		{
			std::cout << "WARNING: config�ļ�����δ֪���ԣ�" << s << std::endl;
			system("pause");
			exit(0);
			break;
		}
	}
	file.close();
}

// ��������ļ�
void CheckConfig()
{
	bool bWarning = false;
	if (windowHeight > 2048 || windowWidth > 2048 || windowHeight < 256 || windowWidth < 256)
	{
		std::cout << "WARNING: config������Խ�磺ScreenConfig" << std::endl;
		bWarning = true;
	}
	if (gridNumX > 64 || gridNumY > 64 || gridNumX < 5 || gridNumY < 5)
	{
		std::cout << "WARNING: config������Խ�磺GridNum" << std::endl;
		bWarning = true;
	}
	if (gridLineWidth > 3 || gridLineWidth < 1)
	{
		std::cout << "WARNING: config������Խ�磺GridLineWidth" << std::endl;
		bWarning = true;
	}

	if (bWarning)
	{
		system("pause");
		exit(0);
	}
}

// ���¿�ʼ
void Restart()
{
	system("cls");
	ReadConfig();
	CheckConfig();
	Initialization(windowHeight, windowWidth, gridNumX, gridNumY);
	startGrid = NULL;
	goalGrid = NULL;
	route.clear();
}

// ������
int main()
{
	int frameCount = 0;
	bool bFinish = false;


	// ��ȡ�����ļ�
	ReadConfig();
	CheckConfig();

	// ��ʼ��
	Initialization(windowHeight, windowWidth, gridNumX, gridNumY);

	int key = -1;
	while (key != 27) // ���û�а�ESC��һֱѭ��
	{
		// ������������
		for (auto& grid : grids)
		{
			grid.Draw(frameBuffer);
		}

		// �����ʼ��ͽ����㶼ָ���ˣ���ʼѰ·
		if (startGrid && goalGrid)
		{
			bool bSuccess = aStarFunc();
			if (bSuccess) // ���Ѱ·�ɹ�
			{
				AddAllRouteGridToVector(goalGrid);
				ShowRoute();
			}

			// ����
			for (auto& grid : grids)
			{
				grid.Draw(frameBuffer);
			}
			bFinish = true;
		}

		// OpenCV����ʾ
		cv::Mat window = cv::Mat(windowHeight, windowWidth, CV_32FC3, frameBuffer.data());
		cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
		cv::namedWindow("A*", cv::WINDOW_KEEPRATIO);
		cv::setMouseCallback("A*", mouse_handler, nullptr);

		frameCount++;
		// std::cout << frameCount << std::endl;

		cv::imshow("A*", window);

		key = cv::waitKey(1);

		// ���Ѱ·���
		if (bFinish)
		{
			system("pause");
			// ��������������¿�ʼ
			Restart();
			frameCount = 0;
			bFinish = false;
		}
	}
	return 0;
}