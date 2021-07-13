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
* 单线程A*算法实现
* 由于使用了OpenCV库，所以没有配置OpenCV则无法编译
* VS2019 OpenCV配置方式：https://1keven1.github.io/2021/01/29/%E3%80%90C-%E3%80%91VS2019%E9%85%8D%E7%BD%AEOpenCV%E5%B9%B6%E8%BF%9B%E8%A1%8C%E6%B5%8B%E8%AF%95/
* 一切坐标以左上角为原点，从0开始计数，横向X轴纵向Y轴
*/

// 全局变量
int windowHeight;		// 屏幕高度
int windowWidth;		// 屏幕宽度
int gridNumX;			// 网格列数
int gridNumY;			// 网格行数
int gridLineWidth;		// 网格边线宽度

// 常量
const cv::Vec3f backgroundColor = cv::Vec3f(0.2, 0.2, 0.2);				// 背景颜色
const cv::Vec3f gridLineColor = cv::Vec3f(1, 1, 1);						// 网格边线颜色
const cv::Vec3f normalGridBackgroundColor = cv::Vec3f(0.1, 0.1, 0.1);	// 普通网格颜色
const cv::Vec3f blockColor = cv::Vec3f(0.95, 0.95, 0.95);				// 不可通过网格颜色
const cv::Vec3f startColor = cv::Vec3f(0, 1, 0);						// 开始网格颜色
const cv::Vec3f goalColor = cv::Vec3f(1, 0, 1);							// 目标网格颜色
const cv::Vec3f routeColor = cv::Vec3f(0.7, 0.5, 0);					// 路径网格颜色

// 通过屏幕坐标获取像素点数组下标
int GetFrameBufferNumByCoord(int x, int y)
{
	return y * windowWidth + x;
}

// 网格种类
enum GridType
{
	NORMAL,
	BLOCK,
	START,
	GOAL,
	ROUTE
};

// 网格结构体
struct Grid
{
public:
	// 构造函数
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

	// 设置网格编号
	void SetIndex(const int& x, const int& y)
	{
		_x = x;
		_y = y;
	}

	// 设置网格可通过性
	void SetBlock(const bool& b)
	{
		bBlock = b;
		if (b) _type = BLOCK;
		else _type = NORMAL;
	}

	// 设置网格数据
	void SetData(const float& x, const float& y, const float& width, const float& height)
	{
		_centerX = x;
		_centerY = y;
		_width = width;
		_height = height;
	}

	// 设置网格种类
	void SetType(const GridType& type)
	{
		_type = type;
	}

	// 设置网格父网格
	void SetParent(Grid* const parent)
	{
		_parent = parent;
	}

	// TODO: 将计算cost放在结构体外
	// 设置网格Cost
	void SetCost4Way(Grid* const goal)
	{
		// GCost
		if (_type == START) // 如果是开始网格
		{
			_gCost = 0;
		}
		else // 如果不是开始网格
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
		if (_type == START) // 如果是开始网格
		{
			_gCost = 0;
		}
		else // 如果不是开始网格
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

	// 获取网格可通过性
	const bool GetBlock()const
	{
		return bBlock;
	}

	// 获取网格种类
	const GridType GetType()const
	{
		return _type;
	}

	// 获取网格GCost
	const int GetGCost()const
	{
		return _gCost;
	}

	// 获取网格HCost
	const int GetHCost()const
	{
		return _hCost;
	}

	// 获取网格FCost
	const int GetFCost()const
	{
		return _fCost;
	}

	// 获取网格父网格
	Grid* const GetParent()const
	{
		return _parent;
	}

	// 获取网格编号
	const std::tuple<float, float> GetIndex()const
	{
		return { _x, _y };
	}

	// 绘制网格
	void Draw(std::vector<cv::Vec3f>& frameBuffer)const
	{
		for (int j = (int)(_centerY - _height * 0.5); j < (int)(_centerY + _height * 0.5); j++)
		{
			for (int i = (int)(_centerX - _width * 0.5); i < (int)(_centerX + _width * 0.5); i++)
			{
				// 防止数组越界
				if (j > windowHeight - 1) i = windowHeight - 1;
				if (i > windowWidth - 1) j = windowWidth - 1;
				// 绘制
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
	int _x, _y;					// 网格编号
	float _centerX, _centerY;	// 网格中心点屏幕坐标
	float _width, _height;		// 网格长宽
	GridType _type;				// 网格种类
	bool bBlock;				// 网格可通过性

	// A*算法所用变量
	int _gCost = 0;				// 开始点走过来的代价
	int _hCost = 0;				// 距离目标点的距离
	int _fCost = 0;
	Grid* _parent = NULL;		// 父网格
};

// 显示所需变量
std::vector<Grid> grids;			// 所有网格
std::vector<cv::Vec3f> frameBuffer;	// Frame Buffer
Grid* startGrid = NULL;				// 开始点
Grid* goalGrid = NULL;				// 结束点
std::vector<Grid*> route;			// 路径点
std::function<bool()> aStarFunc;	// 储存A星算法函数的函数指针

// 通过网格编号获取网格数组下标
int GetGridNumByIndex(const int& x, const int& y)
{
	return y * gridNumX + x;
}

// 通过屏幕坐标获取网格编号
std::tuple<int, int> GetGridIndexByCoord(const int& x, const int& y)
{
	float gridWidth = static_cast<float>(windowWidth) / gridNumX;
	float gridHeight = static_cast<float>(windowHeight) / gridNumY;
	int _x, _y;
	_x = floor(x / gridWidth);
	_y = floor(y / gridHeight);
	return { _x, _y };
}

// 通过屏幕坐标设置网格可通过性
void SetGridBlockByCoord(const int& x, const int& y, const bool& bBlock)
{
	int gridNumX, gridNumY;
	std::tie(gridNumX, gridNumY) = GetGridIndexByCoord(x, y);
	grids[GetGridNumByIndex(gridNumX, gridNumY)].SetBlock(bBlock);
}

// 通过网格编号设置网格可通过性
void SetGridBlockByIndex(const int& x, const int& y, const bool& bBlock)
{
	grids[GetGridNumByIndex(x, y)].SetBlock(bBlock);
}

// 通过屏幕坐标切换网格可通过性
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

// 通过屏幕坐标设置网格种类
void SetGridTypeByCoord(const int& x, const int& y, const GridType& type)
{
	int gridNumX, gridNumY;
	std::tie(gridNumX, gridNumY) = GetGridIndexByCoord(x, y);
	grids[GetGridNumByIndex(gridNumX, gridNumY)].SetType(type);
	std::string tp;
	switch (type)
	{
	case NORMAL:
		tp = "正常网格";
		break;
	case START:
		tp = "开始点";
		break;
	case GOAL:
		tp = "目标点";
		break;
	default:
		break;
	}
	std::cout << "将网格（" << gridNumX << ", " << gridNumY << "）设置成" << tp << std::endl;
}

// OpenCV: 鼠标事件句柄
bool bHoldRButton = false;
GridType type;
int mouseX = -1;
int mouseY = -1;
void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
	// 如果鼠标在窗口外则初始化
	if (x < 0 || y < 0 || x > windowWidth || y > windowHeight)
	{
		bHoldRButton = false;
		mouseX = -1;
		mouseY = -1;
		return;
	}

	// 按下鼠标左键
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		int gridIndexX, gridIndexY;
		std::tie(gridIndexX, gridIndexY) = GetGridIndexByCoord(x, y);
		int gridNum = GetGridNumByIndex(gridIndexX, gridIndexY);
		// 对于不同种类的网格
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

	// 按下鼠标右键
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

	// 松开鼠标右键
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

	// 鼠标移动
	if (event == cv::EVENT_MOUSEMOVE)
	{
		if (bHoldRButton)
		{
			int gridIndexX, gridIndexY;
			std::tie(gridIndexX, gridIndexY) = GetGridIndexByCoord(x, y);
			int gridNum = GetGridNumByIndex(gridIndexX, gridIndexY);
			bool bBlock = type == BLOCK ? false : true;
			// 对于不同种类的网格
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

// 初始化Frame Buffer和网格数组
void Initialization(const int& windowX, const int& windowY, const int& gridX, const int& gridY)
{
	// 初始化Frame Buffer
	frameBuffer.resize(windowX * windowY);
	std::fill(frameBuffer.begin(), frameBuffer.end(), backgroundColor);
	// 初始化Grids数组
	grids.resize(gridX * gridY);
	std::fill(grids.begin(), grids.end(), Grid());
	// 设置Grid的数据
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

// A*算法：数组中是否包含元素
bool VectorContainItem(std::vector<Grid*> vector, Grid* const item)
{
	for (auto _item : vector)
	{
		if (_item == item) return true;
	}
	return false;
}

// A*算法：获取所有邻居
std::vector<Grid*> FindAllNeighbors4Way(Grid* const current)
{
	std::vector<Grid*> neighbors;
	int currentX, currentY;
	std::tie(currentX, currentY) = current->GetIndex();
	// 上
	if (currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// 下
	if (currentY < gridNumY - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY + 1)];
		neighbors.push_back(neighbor);
	}

	// 左
	if (currentX > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY)];
		neighbors.push_back(neighbor);
	}

	// 右
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
	// 上
	if (currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// 下
	if (currentY < gridNumY - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX, currentY + 1)];
		neighbors.push_back(neighbor);
	}

	// 左
	if (currentX > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY)];
		neighbors.push_back(neighbor);
	}

	// 右
	if (currentX < gridNumX - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX + 1, currentY)];
		neighbors.push_back(neighbor);
	}

	// 左上
	if (currentX > 0 && currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// 左下
	if (currentX > 0 && currentY < gridNumY - 1)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX - 1, currentY + 1)];
		neighbors.push_back(neighbor);
	}

	// 右上
	if (currentX < gridNumX - 1 && currentY > 0)
	{
		Grid* neighbor = &grids[GetGridNumByIndex(currentX + 1, currentY - 1)];
		neighbors.push_back(neighbor);
	}

	// 右下
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

// A*算法在这
bool AStar4Way()
{
	// A*算法所需变量
	std::vector<Grid*> openGrid;	// openList
	std::vector<Grid*> closeGrid;	// closeList
	Grid* current = NULL;			// 现在计算的节点

	std::cout << "开始四连通寻路" << std::endl;
	clock_t start = clock();

	// 将开始节点放入Open List
	startGrid->SetCost4Way(goalGrid);
	openGrid.push_back(startGrid);

	// 在Open List中有元素的时候进行循环
	while (openGrid.size() > 0)
	{
		// 将current网格设置成Open List中FCost最小的，如果FCost一样的话设置成HCost最小的
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

		// 将current从open list中移除，加入到close list中
		openGrid.erase(openGrid.begin() + currentNum);
		closeGrid.push_back(current);

		// 如果current是目标点则寻路成功
		if (current && current->GetType() == GOAL)
		{
			clock_t end = clock();
			std::cout << "寻路成功，用时：" << end - start << "毫秒" << std::endl;
			return true;
		}

		// 对于每一个邻居网格
		auto neighbors = FindAllNeighbors4Way(current);
		for (auto neighbor : neighbors)
		{
			// 如果在open和close list中或者是墙，则跳过
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
	std::cout << "失败嘞" << std::endl;
	return false;
}
bool AStar8Way()
{
	// A*算法所需变量
	std::vector<Grid*> openGrid;	// openList
	std::vector<Grid*> closeGrid;	// closeList
	Grid* current = NULL;			// 现在计算的节点

	std::cout << "开始八连通寻路" << std::endl;
	clock_t start = clock();

	// 将开始节点放入Open List
	startGrid->SetCost8Way(goalGrid);
	openGrid.push_back(startGrid);

	// 在Open List中有元素的时候进行循环
	while (openGrid.size() > 0)
	{
		// 将current网格设置成Open List中FCost最小的，如果FCost一样的话设置成HCost最小的
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

		// 将current从open list中移除，加入到close list中
		openGrid.erase(openGrid.begin() + currentNum);
		closeGrid.push_back(current);

		// 如果current是目标点则寻路成功
		if (current && current->GetType() == GOAL)
		{
			clock_t end = clock();
			std::cout << "寻路成功，用时：" << end - start << "毫秒" << std::endl;
			return true;
		}

		// 对于每一个邻居网格
		auto neighbors = FindAllNeighbors8Way(current);
		for (auto neighbor : neighbors)
		{
			if (neighbor->GetBlock() || VectorContainItem(closeGrid, neighbor))
				continue;

			// 如果该路径FCost更低的话更新该节点的Cost
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
	std::cout << "失败嘞" << std::endl;
	return false;
}

// 回溯所有路径节点，储存在数组里
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

// 显示路径点
void ShowRoute()
{
	for (auto routeGrid : route)
	{
		if (routeGrid->GetType() != GOAL) routeGrid->SetType(ROUTE);
	}
}

// 读取配置文件
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
		std::cout << "WARNING: 读取config文件失败" << std::endl;
		system("pause");
		exit(0);
	}

	while (1)
	{
		if (file.eof())
		{
			std::cout << "WARNING: config文件无结束指令\"END\"" << std::endl;
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
				std::cout << "WARNING: config：属性错误：4WayOr8Way" << std::endl;
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
				std::cout << "WARNING: config：丢失属性：ScreenConfig" << std::endl;
				bSuccess = false;
			}
			if (!bReadGridNum)
			{
				std::cout << "WARNING: config：丢失属性：GridNum" << std::endl;
				bSuccess = false;
			}
			if (!bReadGridLineWidth)
			{
				std::cout << "WARNING: config：丢失属性：GridLineWidth" << std::endl;
				bSuccess = false;
			}
			if (!bReadWay)
			{
				std::cout << "WARNING: config：丢失属性：4WayOr8Way" << std::endl;
				bSuccess = false;
			}

			if (bSuccess)
			{
				std::cout << "读取Config文件成功" << std::endl;

				if (b4Way) std::cout << "四连通A*寻路" << std::endl;
				else std::cout << "八连通A*寻路" << std::endl;
				std::cout << "屏幕宽高：" << windowWidth << "x" << windowHeight << std::endl;
				std::cout << "网格数量：" << gridNumX << "x" << gridNumY << std::endl;
				std::cout << "网格线粗细：" << gridLineWidth << std::endl;
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
			std::cout << "WARNING: config文件中有未知属性：" << s << std::endl;
			system("pause");
			exit(0);
			break;
		}
	}
	file.close();
}

// 检查配置文件
void CheckConfig()
{
	bool bWarning = false;
	if (windowHeight > 2048 || windowWidth > 2048 || windowHeight < 256 || windowWidth < 256)
	{
		std::cout << "WARNING: config：属性越界：ScreenConfig" << std::endl;
		bWarning = true;
	}
	if (gridNumX > 64 || gridNumY > 64 || gridNumX < 5 || gridNumY < 5)
	{
		std::cout << "WARNING: config：属性越界：GridNum" << std::endl;
		bWarning = true;
	}
	if (gridLineWidth > 3 || gridLineWidth < 1)
	{
		std::cout << "WARNING: config：属性越界：GridLineWidth" << std::endl;
		bWarning = true;
	}

	if (bWarning)
	{
		system("pause");
		exit(0);
	}
}

// 重新开始
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

// 主函数
int main()
{
	int frameCount = 0;
	bool bFinish = false;


	// 读取配置文件
	ReadConfig();
	CheckConfig();

	// 初始化
	Initialization(windowHeight, windowWidth, gridNumX, gridNumY);

	int key = -1;
	while (key != 27) // 如果没有按ESC就一直循环
	{
		// 绘制所有网格
		for (auto& grid : grids)
		{
			grid.Draw(frameBuffer);
		}

		// 如果开始点和结束点都指定了，开始寻路
		if (startGrid && goalGrid)
		{
			bool bSuccess = aStarFunc();
			if (bSuccess) // 如果寻路成功
			{
				AddAllRouteGridToVector(goalGrid);
				ShowRoute();
			}

			// 绘制
			for (auto& grid : grids)
			{
				grid.Draw(frameBuffer);
			}
			bFinish = true;
		}

		// OpenCV：显示
		cv::Mat window = cv::Mat(windowHeight, windowWidth, CV_32FC3, frameBuffer.data());
		cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
		cv::namedWindow("A*", cv::WINDOW_KEEPRATIO);
		cv::setMouseCallback("A*", mouse_handler, nullptr);

		frameCount++;
		// std::cout << frameCount << std::endl;

		cv::imshow("A*", window);

		key = cv::waitKey(1);

		// 如果寻路完成
		if (bFinish)
		{
			system("pause");
			// 按下任意键，重新开始
			Restart();
			frameCount = 0;
			bFinish = false;
		}
	}
	return 0;
}