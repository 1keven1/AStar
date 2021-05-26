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
* ����ʹ����OpenCV�⣬����û������OpenCV���޷����뱾����
* VS2019 OpenCV���÷�ʽ��https://1keven1.github.io/2021/01/29/%E3%80%90C-%E3%80%91VS2019%E9%85%8D%E7%BD%AEOpenCV%E5%B9%B6%E8%BF%9B%E8%A1%8C%E6%B5%8B%E8%AF%95/
* ����ͨA*�㷨ʵ��
* һ�����������Ͻ�Ϊԭ�㣬��0��ʼ����������X������Y��
*/

// ����
int windowX;	// ��Ļ����
int windowY;	// ��Ļ�߶�
int gridNumX;	// ��������
int gridNumY;	// ��������

constexpr float gridLineWidth = 2;										// ������߿���
const cv::Vec3f backgroundColor = cv::Vec3f(0.2, 0.2, 0.2);				// ������ɫ
const cv::Vec3f gridLineColor = cv::Vec3f(1, 1, 1);						// ���������ɫ
const cv::Vec3f normalGridBackgroundColor = cv::Vec3f(0.1, 0.1, 0.1);	// ��ͨ������ɫ
const cv::Vec3f blockColor = cv::Vec3f(1, 0, 0);						// ����ͨ��������ɫ
const cv::Vec3f startColor = cv::Vec3f(0, 1, 0);						// ��ʼ������ɫ
const cv::Vec3f goalColor = cv::Vec3f(1, 0, 1);							// Ŀ��������ɫ
const cv::Vec3f routeColor = cv::Vec3f(0.7, 0.5, 0);					// ·��������ɫ

// ͨ����Ļ�����ȡ���ص������±�
int GetFrameBufferNumByCoord(int x, int y)
{
	return y * windowX + x;
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
	void SetIndex(int x, int y)
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
	void SetData(float x, float y, float width, float height)
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
	void SetParent(Grid* parent)
	{
		_parent = parent;
	}

	// ��������Cost
	void SetCost(Grid* goal)
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

	// ��ȡ�����ͨ����
	bool GetBlock()
	{
		return bBlock;
	}

	// ��ȡ��������
	GridType GetType()
	{
		return _type;
	}

	// ��ȡ����GCost
	float GetGCost()
	{
		return _gCost;
	}

	// ��ȡ����HCost
	float GetHCost()
	{
		return _hCost;
	}

	// ��ȡ����FCost
	float GetFCost()
	{
		return _fCost;
	}

	// ��ȡ��������
	Grid* GetParent()
	{
		return _parent;
	}

	// ��ȡ������
	std::tuple<float, float> GetIndex()
	{
		return { _x, _y };
	}

	// ��������
	void Draw(std::vector<cv::Vec3f>& frameBuffer)
	{
		for (int j = (int)(_centerY - _height * 0.5); j < (int)(_centerY + _height * 0.5); j++)
		{
			for (int i = (int)(_centerX - _width * 0.5); i < (int)(_centerX + _width * 0.5); i++)
			{
				// ��ֹ����Խ��
				if (i > windowX - 1) i = windowX - 1;
				if (j > windowY - 1) j = windowY - 1;
				// ����
				switch (_type)
				{
				case NORMAL:
					if(abs(i - _centerX) > _width * 0.5 - gridLineWidth || abs(j - _centerY) > _height * 0.5 - gridLineWidth) 
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

// ??ͨ�������Ż�ȡ���������±�
int GetGridNumByIndex(const int& x, const int& y)
{
	return y * gridNumX + x;
}

// ͨ����Ļ�����ȡ������
std::tuple<int, int> GetGridIndexByCoord(const int& x, const int& y)
{
	float gridWidth = static_cast<float>(windowX) / gridNumX;
	float gridHeight = static_cast<float>(windowY) / gridNumY;
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

// ??ͨ�����������������ͨ����
void SetGridBlockByIndex(const int& x, const int& y, const bool& bBlock)
{
	grids[GetGridNumByIndex(x, y)].SetBlock(bBlock);
}

// ??ͨ����Ļ�����л������ͨ����
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

// ??ͨ����Ļ����������������
void SetGridTypeByCoord(const int& x, const int& y, const GridType& type)
{
	int gridNumX, gridNumY;
	std::tie(gridNumX, gridNumY) = GetGridIndexByCoord(x, y);
	grids[GetGridNumByIndex(gridNumX, gridNumY)].SetType(type);
	std::string tp;
	switch (type)
	{
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
void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
	// ���������
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
	// �������Ҽ�
	if (event == cv::EVENT_RBUTTONDOWN)
	{
		int gridIndexX, gridIndexY;
		std::tie(gridIndexX, gridIndexY) = GetGridIndexByCoord(x, y);
		int gridNum = GetGridNumByIndex(gridIndexX, gridIndexY);
		// ���ڲ�ͬ���������
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
	}
}

void ReadConfig()
{
	std::ifstream file("config.txt", std::ios::in);
	if (!file)
	{
		std::cout << "��ȡconfig�ļ�ʧ��" << std::endl;
		system("pause");
	}
	std::string s;
	file >> s;
	file >> windowX >> windowY;
	file >> s;
	file >> gridNumX >> gridNumY;
	file.close();
}

// ��ʼ��Frame Buffer����������
void initialization(int windowX, int windowY, int gridX, int gridY)
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
			float width = static_cast<float>(windowX) / gridX;
			float height = static_cast<float>(windowY) / gridY;
			grids[GetGridNumByIndex(i, j)].SetIndex(i, j);
			grids[GetGridNumByIndex(i, j)].SetData(width * (float)(i + 0.5f), height * (float)(j + 0.5f), width, height);
		}
	}
}

// A*�㷨���������Ƿ����Ԫ��
bool VectorContainItem(std::vector<Grid*> vector, Grid* item)
{
	for (auto _item : vector)
	{
		if (_item == item) return true;
	}
	return false;
}

// A*�㷨����ȡ�����ھ�
std::vector<Grid*> FindAllNeighbors(Grid* current)
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

// A*�㷨����
bool AStar()
{
	// A*�㷨�������
	std::vector<Grid*> openGrid;	// openList
	std::vector<Grid*> closeGrid;	// closeList
	Grid* current = NULL;			// ���ڼ���Ľڵ�

	std::cout << "��ʼѰ·" << std::endl;
	clock_t start = clock();

	// ����ʼ�ڵ����Open List
	startGrid->SetCost(goalGrid);
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
		if (current->GetType() == GOAL)
		{
			clock_t end = clock();
			std::cout << "Ѱ·�ɹ�����ʱ��" << end - start << "����" << std::endl;
			return true;
		}

		// ����ÿһ���ھ�����
		auto neighbors = FindAllNeighbors(current);
		for (auto neighbor : neighbors)
		{
			// �����open��close list�л�����ǽ��������
			if (neighbor->GetBlock() || VectorContainItem(closeGrid, neighbor) || VectorContainItem(openGrid, neighbor)) 
				continue;
			else
			{
				neighbor->SetParent(current);
				neighbor->SetCost(goalGrid);
				openGrid.push_back(neighbor);
			}
		}

	}
	std::cout << "ʧ����" << std::endl;
	return false;
}

// ��������·���ڵ㣬������������
void AddAllRouteGridToVector(Grid* grid)
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
		if(routeGrid->GetType() != GOAL) routeGrid->SetType(ROUTE);
	}
}

// ���¿�ʼ
void Restart()
{
	system("cls");
	initialization(windowX, windowY, gridNumX, gridNumY);
	startGrid = NULL;
	goalGrid = NULL;
	route.clear();
}

// ������
int main()
{
	int frameCount = 0;
	bool bFinish = false;

	ReadConfig();

	// ��ʼ��
	initialization(windowX, windowY, gridNumX, gridNumY);

	int key = -1;
	while (key != 27) // ���û�а�ESC
	{
		// ������������
		for (auto& grid : grids)
		{
			grid.Draw(frameBuffer);
		}

		// �����ʼ��ͽ����㶼ָ���ˣ���ʼѰ·
		if (startGrid && goalGrid)
		{
			bool bSuccess = AStar();
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
		cv::Mat window = cv::Mat(windowX, windowY, CV_32FC3, frameBuffer.data());
		cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
		cv::namedWindow("A*", cv::WINDOW_AUTOSIZE);
		cv::setMouseCallback("A*", mouse_handler, nullptr);

		frameCount ++;
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