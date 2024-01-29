#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <complex>
#include <chrono>
#include <memory>
#include <unordered_set>

using namespace std::chrono;
using namespace std;

struct ShaderProgramSource
{
    string VertexSource;
    string FragmentSource;
};

static ShaderProgramSource ParseShader(const string& filepath)
{
    ifstream stream(filepath);

    enum class ShaderType
    {
        NONE = -1, VERTEX = 0, FRAGMENT = 1
    };

    string line;
    stringstream ss[2];
    ShaderType type = ShaderType::NONE;
    while (getline(stream, line))
    {
        if (line.find("#shader") != string::npos)
        {
            if (line.find("vertex") != string::npos)
                type = ShaderType::VERTEX;
            else if (line.find("fragment") != string::npos)
                type = ShaderType::FRAGMENT;
        }
        else
        {
            ss[(int)type] << line << '\n';
        }
    }
    return { ss[0].str(), ss[1].str() };
}

static unsigned int CompileShader(unsigned int type, const string& source)
{
    unsigned int id = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int result;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE)
    {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        glGetShaderInfoLog(id, length, &length, message);
        cout << "Failed to compile " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!" << endl;
        cout << message << endl;
        glDeleteShader(id);
        return 0;
    }
    return id;
}

static int CreateShader(const string& vertexShader, const string& fragmentShader)
{
    unsigned int program = glCreateProgram();
    unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}

const int width = 960;
const int height = 960;
const float M_PI = 3.14159265358979323846;
const float M_PI2 = 2 * M_PI;
const int PointsPerCircle = 50;


unsigned int circleIndicesSize;
unsigned int* circleIndices;

struct vec2
{
    float x, y;

    vec2(){}

    vec2(float xi, float yi)
    {
        x = xi;
        y = yi;
    }

    vec2 operator +(vec2 v)
    {
        return vec2(x + v.x, y + v.y);
    }

    vec2 operator -(vec2 v)
    {
        return vec2(x - v.x, y - v.y);
    }

    vec2 operator *(vec2 v)
    {
        return vec2(x * v.x, y * v.y);
    }

    vec2 operator /(vec2 v)
    {
        return vec2(x / v.x, y / v.y);
    }

    vec2 operator *(float c)
    {
        return vec2(x * c, y * c);
    }

    vec2 operator /(float c)
    {
        return vec2(x / c, y / c);
    }

    bool operator ==(vec2 v)
    {
        return (x == v.x && y == v.y);
    }

    void operator +=(vec2 v)
    {
        x += v.x;
        y += v.y;
    }

    void operator -=(vec2 v)
    {
        x -= v.x;
        y -= v.y;
    }

    float norm()
    {
        return sqrt(x * x + y * y);
    }
};

vec2 normalize(vec2 v)
{
    float norm = v.norm();
    v.x /= norm;
    v.y /= norm;
    return v;
}

float crossProduct(vec2 p1, vec2 p2)
{
    return p1.x * p2.y - p1.y * p2.x;
}

float dot(vec2 p1, vec2 p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}

void CreateCircleVertices(vec2 center, float radius, vec2 *vertices)
{
    vertices[0] = center;
    float theta = 0;
    float thetaStep = M_PI2/(PointsPerCircle - 1);
    for (int i = 0; i < PointsPerCircle; i++)
    {
        vertices[i + 1] = center + vec2(cos(theta), sin(theta)) * radius;
        theta += thetaStep;
    }
}

unsigned int* CreateCircleIndices(unsigned int& size)
{
    size = 3 * PointsPerCircle;
    unsigned int* indices = new unsigned int[size];
    for (int i = 0; i < PointsPerCircle - 1; i++)
    {
        indices[i * 3] = 0;
        indices[i * 3 + 1] = i + 1;
        indices[i * 3 + 2] = i + 2;
    }
    indices[(PointsPerCircle - 1) * 3] = 0;
    indices[(PointsPerCircle - 1) * 3 + 1] = PointsPerCircle;
    indices[(PointsPerCircle - 1) * 3 + 2] = 1;
    return indices;
}

class obstacle
{
public:
    int verticesNeeded = 0;
    int indicesNeeded = 0;
    virtual  bool insideObstacle(vec2 point) { return false; }
    virtual void createVertices(vec2* arr) {}
    virtual void createIndices(unsigned int* arr, int indexOffset) {}
};

int triangleIndices[] = { 0, 1, 2 };

class triangle : public obstacle
{
public:
    vec2* vertices;
    vec2* edges;
    triangle(vec2 p1, vec2 p2, vec2 p3)
    {
        verticesNeeded = 3;
        indicesNeeded = 3;
        vertices = new vec2[3]{ p1, p2, p3 };
        edges = new vec2[3]{
            vertices[1] - vertices[0],
            vertices[2] - vertices[1],
            vertices[0] - vertices[2]
        };
    }

    bool insideObstacle(vec2 point)
    {
        float ABxAP = crossProduct(edges[0], point - vertices[0]);
        float BCxBP = crossProduct(edges[1], point - vertices[1]);
        float CAxCP = crossProduct(edges[2], point - vertices[2]);

        return ((ABxAP <= 0 && BCxBP <= 0 && CAxCP <= 0) || (ABxAP > 0 && BCxBP > 0 && CAxCP > 0));
    }

    void createVertices(vec2* arr)
    {
        for (int i = 0; i < 3; i++)
        {
            arr[i] = vertices[i];
        }
    }

    void createIndices(unsigned int* arr, int indexOffset)
    {
        for (int i = 0; i < 3; i++)
        {
            arr[i] = indexOffset + i;
        }
    }
};

class circle : public obstacle
{
public:
    vec2 center;
    float radius;
    circle(vec2 c, float r)
    {
        verticesNeeded = PointsPerCircle + 1;
        indicesNeeded = PointsPerCircle * 3;
        center = c;
        radius = r;
    }

    bool insideObstacle(vec2 point)
    {
        return (point - center).norm() < radius;
    }

    void createVertices(vec2* arr)
    {
        CreateCircleVertices(center, radius, arr);
    }

    void createIndices(unsigned int* arr, int indexOffset)
    {
        for (int i = 0; i < circleIndicesSize; i++)
        {
            arr[i] = circleIndices[i] + indexOffset;
        }
    }
};

struct obstacleWorld
{
    vec2* vertices;
    unsigned int* indices;
    unsigned int verticesSize;
    unsigned int indicesSize;

    obstacleWorld(vector<obstacle*> obstacleList)
    {
        verticesSize = 0;
        indicesSize = 0;
        for (const auto &currObstacle : obstacleList)
        {
            verticesSize += currObstacle -> verticesNeeded;
            indicesSize += currObstacle -> indicesNeeded;
        }
        vertices = new vec2[verticesSize];
        indices = new unsigned int[indicesSize];
        int curriVertex = 0;
        int curriIndex = 0;

        for (auto &currObstacle : obstacleList)
        {
            currObstacle -> createVertices(&vertices[curriVertex]);
            currObstacle -> createIndices(&indices[curriIndex], curriVertex);
            curriVertex += currObstacle -> verticesNeeded;
            curriIndex += currObstacle -> indicesNeeded;
        }
    }
};

vec2 robotCenter(0.4, 0.6);
float robotRadius = 0.02;
float robotVisionRadius = 0.1;
vector<obstacle*> obstacleList;
vector<vec2> worldPath;
vector<float> worldThresholds;
vec2 movingTowards = robotCenter;
vec2* robotPositions;
float raySpeed = 0.01 / 4;
bool lbuttonDown = false;
double mouseX, mouseY;


bool raycast(vec2 origin, vec2 direction, float r, vector<obstacle*> &obstacleList, vec2 &hitPoint)
{
    hitPoint = origin;
    vec2 step = direction * raySpeed;
    for (int i = 0; i < r / raySpeed; i++)
    {
        hitPoint += step;
        if (fabs(hitPoint.x) >= 1 || fabs(hitPoint.y) >= 1)
        {
            return true;
        }

        for (auto obs : obstacleList)
        {
            if (obs->insideObstacle(hitPoint))
            {
                return true;
            }
        }
    }
    return false;
}

bool collinear(vec2 p1, vec2 p2, vec2 p3)
{
    return fabs((p2.y - p1.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p2.x - p1.x)) < 0.001;
}

float findAngle(vec2 o, vec2 p1, vec2 p2)
{
    vec2 v1 = normalize(p2 - o);
    vec2 v2 = normalize(p1 - o);

    float ans = atan2(crossProduct(v1, v2), dot(v1, v2));
    return ans < 0 ? ans + M_PI2 : ans;
}

void findThresholds()
{
    worldThresholds[0] = findAngle(worldPath[0], worldPath[worldPath.size() - 1], worldPath[1]) / 2;
    for (int i = 1; i < worldPath.size() - 1; i++)
    {
        worldThresholds[i] = findAngle(worldPath[i], worldPath[i - 1], worldPath[i + 1]) / 2;
    }
    worldThresholds[worldPath.size() - 1] = findAngle(worldPath[worldPath.size() - 1], worldPath[worldPath.size() - 2], worldPath[0]) / 2;
}

int CreateVisibilityPolygon(vec2 viewPoint, obstacleWorld obstacles, vec2* polygon)
{
    polygon[0] = viewPoint;
    int polygonIdx = 1;
    for (int i = 0; i < worldPath.size(); i++)
    {
        vec2 dir = worldPath[i] - viewPoint;
        float dist = dir.norm();
        vec2 hitPoint;
        int nexti = (i + 1) % worldPath.size();
        if (collinear(worldPath[i], worldPath[nexti], viewPoint))
        {
            vec2 dir2 = worldPath[nexti] - viewPoint;
            float dist2 = dir2.norm();
            vec2 closestPoint = worldPath[i];
            vec2 mostDistantPoint = worldPath[nexti];
            if (dist2 < dist)
            {
                mostDistantPoint = worldPath[i];
                closestPoint = worldPath[nexti];
                dir = dir2;
                dist = dist2;
            }

            dir = dir / dist;
            raycast(viewPoint, dir, dist - raySpeed, obstacleList, hitPoint);
            if ((hitPoint - closestPoint).norm() > .01)
            {
                i = nexti;
                if (i == 0) break;
                continue;
            }

            polygon[polygonIdx++] = worldPath[i];
            polygon[polygonIdx++] = worldPath[nexti];

            raycast(mostDistantPoint, dir, 8, obstacleList, hitPoint);
            if ((hitPoint - mostDistantPoint).norm() > 2 * raySpeed)
            {
                if (mostDistantPoint == worldPath[i])
                {
                    polygon[polygonIdx - 1] = mostDistantPoint;
                    polygon[polygonIdx - 2] = hitPoint;
                    polygon[polygonIdx++] = worldPath[nexti];
                }
                else
                    polygon[polygonIdx++] = hitPoint;
            }

            i = nexti;
            if (i == 0) break;

            continue;
        }
        dir = dir / dist;
        raycast(viewPoint, dir, dist - raySpeed, obstacleList, hitPoint);
        if ((hitPoint - worldPath[i]).norm() > .01)
        {
            continue;
        }

        polygon[polygonIdx++] = worldPath[i];

        raycast(worldPath[i], dir, 8, obstacleList, hitPoint);
        if ((hitPoint - worldPath[i]).norm() > 2 * raySpeed)
        {
            if (findAngle(worldPath[i], viewPoint, worldPath[(i + 1) % worldPath.size()]) < worldThresholds[i])
            {
                polygon[polygonIdx] = worldPath[i];
                polygon[polygonIdx - 1] = hitPoint;
                polygonIdx++;
            }
            else
                polygon[polygonIdx++] = hitPoint;
        }
    }

    return polygonIdx;
}

unsigned int* CreateVisibilityPolygonIndices(int polygonSize, int &size)
{
    size = 3 * (polygonSize - 1);
    unsigned int* indices = new unsigned int[size];
    for (int i = 0; i < polygonSize - 2; i++)
    {
        indices[i * 3] = 0;
        indices[i * 3 + 1] = i + 1;
        indices[i * 3 + 2] = i + 2;
    }
    indices[(polygonSize - 2) * 3] = 0;
    indices[(polygonSize - 2) * 3 + 1] = polygonSize - 1;
    indices[(polygonSize - 2) * 3 + 2] = 1;

    return indices;
}

void ManageMouseInput(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
        lbuttonDown = true;
    else
        lbuttonDown = false;
}

vec2 robotTranslation(0, 0);

void updateRobot(GLFWwindow* window)
{
    if (lbuttonDown == false)
        return;

    glfwGetCursorPos(window, &mouseX, &mouseY);

    mouseY -= height;
    mouseY *= -1;

    movingTowards = vec2(mouseX * 2.0 / height - 1.0, mouseY * 2.0 / height - 1.0);
    if (movingTowards.x > .98) return;
    if (movingTowards.x < -.98) return;
    if (movingTowards.y > .98) return;
    if (movingTowards.y < -.98) return;
    for (auto obs : obstacleList)
    {
        if (obs->insideObstacle(movingTowards))
            return;
    }
    robotTranslation += movingTowards - robotCenter;
    robotCenter = movingTowards;
    //Translate(robotPositions, PointsPerCircle + 1, translation);
}

int main(void)
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);

    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;


    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(width, height, "Tangent Bug", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    glfwSwapInterval(1);

    if (glewInit() != GLEW_OK)
        cout << "Error!" << endl;

    float screenPositions[] =
    {
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };

    unsigned int screenIndices[] =
    {
        0, 1, 2,
        2, 3, 0
    };

    circleIndices = CreateCircleIndices(circleIndicesSize);

    vec2* robotVisionPositions = new vec2[PointsPerCircle + 1];
    CreateCircleVertices(robotCenter, robotVisionRadius, robotVisionPositions);

    robotPositions = new vec2[PointsPerCircle + 1];
    CreateCircleVertices(robotCenter, robotRadius, robotPositions);

    obstacleList.push_back(new triangle(
        vec2(-1, -1),
        vec2(-.8, 0),
        vec2(-.4, -.2)));

    obstacleList.push_back(new triangle(
        vec2(-1, -1),
        vec2(-.2, -.8),
        vec2(-.4, -.2)));

    obstacleList.push_back(new triangle(
        vec2(-1, -1),
        vec2(-.2, -.8),
        vec2(0, -1)));

    obstacleList.push_back(new triangle(
        vec2(-1, -1),
        vec2(-.8, 0),
        vec2(-1, 0)));

    obstacleList.push_back(new triangle(
        vec2(-.8, .2),
        vec2(-.8, 0),
        vec2(-1, 0)));

    obstacleList.push_back(new triangle(
        vec2(-.8, .2),
        vec2(-1, 1),
        vec2(-1, 0)));

    obstacleList.push_back(new triangle(
        vec2(-.8, .2),
        vec2(-1, 1),
        vec2(-.6, .6)));

    obstacleList.push_back(new triangle(
        vec2(-.2, .8),
        vec2(-1, 1),
        vec2(-.6, .6)));

    obstacleList.push_back(new triangle(
        vec2(-.2, .8),
        vec2(-1, 1),
        vec2(0, 1)));

    obstacleList.push_back(new triangle(
        vec2(-.2, .8),
        vec2(-.4, .2),
        vec2(-.6, .6)));

    obstacleList.push_back(new triangle(
        vec2(-.2, .8),
        vec2(0, 1),
        vec2(.4, .8)));

    obstacleList.push_back(new triangle(
        vec2(-.2, .8),
        vec2(.2, .6),
        vec2(.4, .8)));

    obstacleList.push_back(new triangle(
        vec2(0, 1),
        vec2(1, 1),
        vec2(.4, .8)));

    obstacleList.push_back(new triangle(
        vec2(.6, .6),
        vec2(1, 1),
        vec2(1, 0)));

    obstacleList.push_back(new triangle(
        vec2(.6, .6),
        vec2(.4, .4),
        vec2(1, 0)));

    obstacleList.push_back(new triangle(
        vec2(.6, .6),
        vec2(.4, .8),
        vec2(1, 1)));

    obstacleList.push_back(new triangle(
        vec2(-.2, -.2),
        vec2(0, .2),
        vec2(.2, 0)));

    obstacleList.push_back(new triangle(
        vec2(-.2, -.2),
        vec2(.2, -.4),
        vec2(.2, 0)));

    obstacleList.push_back(new triangle(
        vec2(.4, -.2),
        vec2(.2, -.4),
        vec2(.2, 0)));

    obstacleList.push_back(new triangle(
        vec2(.4, -.2),
        vec2(1, 0),
        vec2(.2, 0)));

    obstacleList.push_back(new triangle(
        vec2(.4, -.2),
        vec2(1, 0),
        vec2(.6, -.4)));

    obstacleList.push_back(new triangle(
        vec2(1, -1),
        vec2(1, 0),
        vec2(.6, -.4)));

    obstacleList.push_back(new triangle(
        vec2(1, -1),
        vec2(.4, -.8),
        vec2(.6, -.4)));

    obstacleList.push_back(new triangle(
        vec2(1, -1),
        vec2(.4, -.8),
        vec2(0, -1)));

    obstacleList.push_back(new triangle(
        vec2(-.2, -.8),
        vec2(.4, -.8),
        vec2(0, -1)));

    obstacleList.push_back(new triangle(
        vec2(-.2, -.8),
        vec2(.4, -.8),
        vec2(0, -.6)));

    obstacleList.push_back(new triangle(
        vec2(.2, 0),
        vec2(.4, .4),
        vec2(1, 0)));

    worldPath = { vec2(-.2, -.8),
                vec2(0, -.6),
                vec2(.4, -0.8),
                vec2(0.6, -.4),
                vec2(.4, -.2),
                vec2(.2, -.4),
                vec2(-0.2, -.2),
                vec2(0, 0.2),
                vec2(.2, 0),
                vec2(0.4, 0.4),
                vec2(.6, .6),
                vec2(.4, .8),
                vec2(0.2, .6),
                vec2(-.2, 0.8),
                vec2(-.4, 0.2),
                vec2(-.6, .6), 
                vec2(-.8, .2),
                vec2(-0.8, 0),
                vec2(-.4, -0.2)};

    worldThresholds = vector<float> (worldPath.size());

    findThresholds();

    obstacleWorld world(obstacleList);

    vec2* visibilityPolygon = new vec2[worldPath.size() + 1];
    int visibilityPolygonSize = CreateVisibilityPolygon(robotCenter, world, visibilityPolygon);

    int visibilityPolygonIndicesSize;
    unsigned int* visibilityPolygonIndices = CreateVisibilityPolygonIndices(visibilityPolygonSize, visibilityPolygonIndicesSize);

    unsigned int screenBuffer;
    unsigned int robotBuffer;
    unsigned int obstacleBuffer;
    unsigned int robotVisionBuffer;
    unsigned int obstacleIbo;
    unsigned int screenIbo;
    unsigned int circleIbo;
    unsigned int robotVisionIbo;

    glGenBuffers(1, &screenBuffer);
    glGenBuffers(1, &robotBuffer);
    glGenBuffers(1, &obstacleBuffer);
    glGenBuffers(1, &robotVisionBuffer);
    glGenBuffers(1, &screenIbo);
    glGenBuffers(1, &circleIbo);
    glGenBuffers(1, &obstacleIbo);
    glGenBuffers(1, &robotVisionIbo);

    glBindBuffer(GL_ARRAY_BUFFER, screenBuffer);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), screenPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, screenIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(unsigned int), screenIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, robotBuffer);
    glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, circleIndicesSize * sizeof(unsigned int), circleIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, robotVisionBuffer);
    glBufferData(GL_ARRAY_BUFFER, visibilityPolygonSize * sizeof(vec2), visibilityPolygon, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, robotVisionIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, visibilityPolygonIndicesSize * sizeof(unsigned int), visibilityPolygonIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, obstacleBuffer);
    glBufferData(GL_ARRAY_BUFFER, world.verticesSize * sizeof(vec2), world.vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obstacleIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, world.indicesSize * sizeof(unsigned int), world.indices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, robotVisionBuffer);
    glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotVisionPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);


    unsigned int fbo, render_buf;
    glGenFramebuffers(1, &fbo);
    glGenRenderbuffers(1, &render_buf);
    glBindRenderbuffer(GL_RENDERBUFFER, render_buf);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, width, height);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, render_buf);

    ShaderProgramSource source = ParseShader("Basic.shader");

    unsigned int shader = CreateShader(source.VertexSource, source.FragmentSource);
    glUseProgram(shader);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    int inputColLocation = glGetUniformLocation(shader, "inputCol");
    if (inputColLocation == -1) return -1;

    cv::Mat M;
    M.create(height, width, CV_8UC3);

    cv::VideoWriter outputVideo;
    int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

    if (!outputVideo.open("video.mp4", codec, 30.0, M.size(), true))
    {
        cout << "Problema al abrir el archivo" << endl;
        return -1;
    }

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);

    glfwSetMouseButtonCallback(window, ManageMouseInput);

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

        //Draw world
        glUniform3f(inputColLocation, 0, 0, 1);
        glUniform2f(glGetUniformLocation(shader, "translation"), 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, screenBuffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, screenIbo);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

        //Draw vision
        glUniform3f(inputColLocation, 1, 1, 0);
        glBindBuffer(GL_ARRAY_BUFFER, robotVisionBuffer);
        glBufferData(GL_ARRAY_BUFFER, visibilityPolygonSize * sizeof(vec2), visibilityPolygon, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, robotVisionIbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, visibilityPolygonIndicesSize * sizeof(unsigned int), visibilityPolygonIndices, GL_STATIC_DRAW);
        glDrawElements(GL_TRIANGLES, visibilityPolygonIndicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw obstacles
        glUniform3f(inputColLocation, 0, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, obstacleBuffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obstacleIbo);
        glDrawElements(GL_TRIANGLES, world.indicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw robot
        glUniform3f(inputColLocation, 1, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, robotBuffer);
        glUniform2f(glGetUniformLocation(shader, "translation"), robotTranslation.x, robotTranslation.y);
        glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotPositions, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
        glDrawElements(GL_TRIANGLES, circleIndicesSize, GL_UNSIGNED_INT, nullptr);

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        //Video

        /*
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);

        //Draw world
        glUniform3f(inputColLocation, 0, 0, 1);
        glUniform2f(glGetUniformLocation(shader, "translation"), 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, screenBuffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, screenIbo);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

        //Draw vision
        glUniform3f(inputColLocation, 1, 1, 0);
        glBindBuffer(GL_ARRAY_BUFFER, robotVisionBuffer);
        glBufferData(GL_ARRAY_BUFFER, visibilityPolygonSize * sizeof(vec2), visibilityPolygon, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, robotVisionIbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, visibilityPolygonIndicesSize * sizeof(unsigned int), visibilityPolygonIndices, GL_STATIC_DRAW);
        glDrawElements(GL_TRIANGLES, visibilityPolygonIndicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw obstacles
        glUniform3f(inputColLocation, 0, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, obstacleBuffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obstacleIbo);
        glDrawElements(GL_TRIANGLES, world.indicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw robot
        glUniform3f(inputColLocation, 1, 0, 0);
        glUniform2f(glGetUniformLocation(shader, "translation"), robotTranslation.x, robotTranslation.y);
        glBindBuffer(GL_ARRAY_BUFFER, robotBuffer);
        glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotPositions, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
        glDrawElements(GL_TRIANGLES, circleIndicesSize, GL_UNSIGNED_INT, nullptr);

        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, M.data);

        cv::flip(M, M, 0);

        outputVideo << M;*/

        //Actualizar el robot
        updateRobot(window);
        visibilityPolygonSize = CreateVisibilityPolygon(robotCenter, world, visibilityPolygon);
        delete[] visibilityPolygonIndices;
        visibilityPolygonIndices = CreateVisibilityPolygonIndices(visibilityPolygonSize, visibilityPolygonIndicesSize);

        /* Poll for and process events */
        //glfwPollEvents();
        glfwWaitEvents();
    }

    delete[] visibilityPolygonIndices;

    glDeleteProgram(shader);

    glfwTerminate();

    return 0;
}