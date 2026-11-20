#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iomanip>
#include <unordered_map>



using namespace std;



class GroundAlreadySetException : public std::exception {
public:
    const char* what() const noexcept override {
        return "Error: Only one ground node allowed!";
    }
};

class NoGroundNodeException : public std::exception {
public:
    const char* what() const noexcept override {
        return "Error: No ground node defined in the circuit!";
    }
};


class Node {
public:
    int number = -1;
    int x = -1, y = -1;
    bool connect = false;
    bool isGround = false;
    string name = "";
    double voltage = 0.0;


    void get_data(int Number, int X, int Y) {
        number = Number;
        x = X;
        y = Y;
        connect = false;
        voltage = 0.0;
    }

    bool isMouseOver(int mx, int my) {
        return (mx >= x - 10 && mx <= x + 10 && my >= y - 10 && my <= y + 10);
    }
};

class Element {
public:
    string name, type;
    int value;
    Node* from = nullptr;
    Node* to = nullptr;

    Element(string n, string t, int v, Node* f, Node* t_)
            : name(n), type(t), value(v), from(f), to(t_) {}
    virtual ~Element() = default;
};

class Resistor : public Element {
public:
    Resistor(string n, int v, Node* f, Node* t_) : Element(n, "resistor", v, f, t_) {}
};

class Capacitor : public Element {
public:
    Capacitor(string n, int v, Node* f, Node* t_) : Element(n, "capacitor", v, f, t_) {}
};

class Inductor : public Element {
public:
    Inductor(string n, int v, Node* f, Node* t_) : Element(n, "inductor", v, f, t_) {}
};

class VoltageSource : public Element {
public:
    VoltageSource(string n, int v, Node* f, Node* t_) : Element(n, "voltageSource", v, f, t_) {}
};

class CurrentSource : public Element {
public:
    CurrentSource(string n, int v, Node* f, Node* t_) : Element(n, "currentSource", v, f, t_) {}
};

class CircuitSolver {
private:
    vector<Element*>& elements;
    Node nodes[7][10];
    unordered_map<Node*, int> nodeIndex;
    vector<Node*> nodeList;
    int voltageSourceCount = 0;

    vector<vector<double>> G, B, C, D, A;
    vector<double> z, x;

public:
    CircuitSolver(vector<Element*>& el, Node n[7][10]) : elements(el) {
        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 10; ++j)
                nodes[i][j] = n[i][j];
    }

    void solve() {
        bool hasGround = false;
        for (auto& row : nodes) {
            for (auto& node : row) {
                if (node.isGround) {
                    hasGround = true;
                    break;
                }
            }
            if (hasGround) break;
        }

        if (!hasGround) {
            throw NoGroundNodeException();
        }
        prepareNodeMap();
        prepareMatrices();
        solveSystem();
        updateNodeVoltages();
        printResults();
    }

private:
    void prepareNodeMap() {
        int index = 0;
        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 10; ++j)
                if (!nodes[i][j].isGround) {
                    nodeIndex[&nodes[i][j]] = index++;
                    nodeList.push_back(&nodes[i][j]);
                }

        for (auto e : elements)
            if (e->type == "V")
                voltageSourceCount++;
    }

    void prepareMatrices() {
        int n = nodeList.size();
        int m = voltageSourceCount;

        G.assign(n, vector<double>(n, 0));
        B.assign(n, vector<double>(m, 0));
        C.assign(m, vector<double>(n, 0));
        D.assign(m, vector<double>(m, 0));
        z.assign(n + m, 0);
        x.assign(n + m, 0);

        int vIndex = 0;
        for (auto e : elements) {
            int i = nodeIndex.count(e->from) ? nodeIndex[e->from] : -1;
            int j = nodeIndex.count(e->to) ? nodeIndex[e->to] : -1;

            if (e->type == "R") {
                double g = 1.0 / e->value;
                if (i != -1) G[i][i] += g;
                if (j != -1) G[j][j] += g;
                if (i != -1 && j != -1) {
                    G[i][j] -= g;
                    G[j][i] -= g;
                }
            } else if (e->type == "V") {
                if (i != -1) {
                    B[i][vIndex] = 1;
                    C[vIndex][i] = 1;
                }
                if (j != -1) {
                    B[j][vIndex] = -1;
                    C[vIndex][j] = -1;
                }
                z[n + vIndex] = e->value;
                vIndex++;
            }
        }

        A.assign(n + m, vector<double>(n + m, 0));
        for (int r = 0; r < n; ++r)
            for (int c = 0; c < n; ++c)
                A[r][c] = G[r][c];

        for (int r = 0; r < n; ++r)
            for (int c = 0; c < m; ++c)
                A[r][n + c] = B[r][c];

        for (int r = 0; r < m; ++r)
            for (int c = 0; c < n; ++c)
                A[n + r][c] = C[r][c];

        for (int r = 0; r < m; ++r)
            for (int c = 0; c < m; ++c)
                A[n + r][n + c] = D[r][c];
    }

    void solveSystem() {
        int N = A.size();
        for (int i = 0; i < N; ++i) {
            int pivot = i;
            for (int j = i + 1; j < N; ++j)
                if (abs(A[j][i]) > abs(A[pivot][i]))
                    pivot = j;

            swap(A[i], A[pivot]);
            swap(z[i], z[pivot]);

            for (int j = i + 1; j < N; ++j) {
                double f = A[j][i] / A[i][i];
                for (int k = i; k < N; ++k)
                    A[j][k] -= f * A[i][k];
                z[j] -= f * z[i];
            }
        }

        for (int i = N - 1; i >= 0; --i) {
            x[i] = z[i];
            for (int j = i + 1; j < N; ++j)
                x[i] -= A[i][j] * x[j];
            x[i] /= A[i][i];
        }
    }

    void updateNodeVoltages() {
        for (int i = 0; i < nodeList.size(); ++i)
            nodeList[i]->voltage = x[i];
    }

    void printResults() {
        cout << "--- Circuit Solution ---\n";
        for (int i = 0; i < nodeList.size(); ++i) {
            cout << "Node " << nodeList[i]->name << " (id=" << nodeList[i]->number << ") voltage = "
                 << fixed << setprecision(2) << nodeList[i]->voltage << " V\n";
        }

        int idx = nodeList.size();
        for (auto e : elements) {
            if (e->type == "V") {
                cout << "Current through " << e->name << " = " << fixed << setprecision(2)
                     << x[idx++] << " A\n";
            } else if (e->type == "R") {
                double v1 = e->from->isGround ? 0 : e->from->voltage;
                double v2 = e->to->isGround ? 0 : e->to->voltage;
                double current = (v1 - v2) / e->value;
                cout << "Current through " << e->name << " = " << fixed << setprecision(2)
                     << current << " A\n";
            }
        }
    }
};



void load_elements_from_file(const string& filename, vector<Element*>& elements, Node nodes[7][10]);


void draw_element(SDL_Renderer* renderer, TTF_Font* font, Element* e);


int main(int argc, char* argv[]) {
    SDL_StartTextInput();
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    SDL_Window* window = SDL_CreateWindow("Circuit Editor", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1100, 800, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    TTF_Font* font = TTF_OpenFont("C:/Windows/Fonts/arial.ttf", 16);

    Node nodes[7][10];
    for (int i = 0; i < 7; ++i)
        for (int j = 0; j < 10; ++j)
            nodes[i][j].get_data(i * 10 + j, (j + 1) * 100, (i + 1) * 100);

    vector<Element*> elements;
    SDL_Rect getFileButton = {30, 730, 180, 40};

    static Element* selectedElement = nullptr;
    SDL_Rect deleteButton = {850, 730, 100, 40};
    SDL_Rect valueButton = {970, 730, 100, 40};

    Node* selectedNode = nullptr;
    SDL_Rect groundButton = {660, 730, 100, 40};
    SDL_Rect nameButton = {770, 730, 100, 40};
    bool nameInputPending = false;
    bool groundTogglePending = false;
    Node* nodeToName = nullptr;


    string typingText = "";
    bool typingMode = false;

    bool valueInputMode = false;
    string valueText = "";



    SDL_Rect solveButton = {230, 730, 60, 40};
    SDL_Rect simpleButton = {310,730,70,40};
    SDL_Rect nemad_elmiButton = {400,730,110,40};
    SDL_Rect vahedButton = {530,730,70,40};
    bool number_simple=false;
    bool number_nemad_elmi=false;
    bool number_vahed=false;

    bool addressInputMode = false;
    string addressText = "";



    bool running = true;
    bool dragging = false;
    Node* draggedNode = nullptr;
    int mouseX = 0, mouseY = 0;


    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;

            else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                SDL_GetMouseState(&mouseX, &mouseY);


                if (mouseX >= getFileButton.x && mouseX <= getFileButton.x + getFileButton.w &&
                    mouseY >= getFileButton.y && mouseY <= getFileButton.y + getFileButton.h) {
                    addressText = "";
                    addressInputMode = true;
                }


                else if (mouseX >= solveButton.x && mouseX <= solveButton.x + solveButton.w &&
                         mouseY >= solveButton.y && mouseY <= solveButton.y + solveButton.h) {
                    number_nemad_elmi= true;
                    number_simple= true;
                    number_vahed= true;
                    try {
                        CircuitSolver solver(elements, nodes);
                        solver.solve();
                    } catch (const exception& e) {
                        cout << e.what() << endl;
                    }
                }

                else if (selectedNode != nullptr &&
                         mouseX >= groundButton.x && mouseX <= groundButton.x + groundButton.w &&
                         mouseY >= groundButton.y && mouseY <= groundButton.y + groundButton.h) {

                    if (!groundTogglePending) {
                        groundTogglePending = true;

                        try {
                            if (selectedNode->isGround) {
                                selectedNode->isGround = false;
                            } else {
                                for (int i = 0; i < 7; ++i)
                                    for (int j = 0; j < 10; ++j)
                                        if (nodes[i][j].isGround)
                                            throw GroundAlreadySetException();
                                selectedNode->isGround = true;
                            }
                        } catch (const exception& e) {
                            cerr << e.what() << endl;
                        }

                        selectedNode = nullptr;
                    }
                }


                else if (selectedNode != nullptr &&
                         mouseX >= nameButton.x && mouseX <= nameButton.x + nameButton.w &&
                         mouseY >= nameButton.y && mouseY <= nameButton.y + nameButton.h) {
                    typingText = "";
                    typingMode = true;
                    nodeToName = selectedNode;
                    selectedNode = nullptr;
                }


                else {
                    selectedNode = nullptr;
                    for (int i = 0; i < 7; ++i) {
                        for (int j = 0; j < 10; ++j) {
                            if (nodes[i][j].isMouseOver(mouseX, mouseY)) {
                                selectedNode = &nodes[i][j];
                                dragging = true;
                                draggedNode = &nodes[i][j];
                                break;
                            }
                        }
                        if (selectedNode) break;
                    }

                    for (Element* e : elements) {
                        int midX = (e->from->x + e->to->x) / 2;
                        int midY = (e->from->y + e->to->y) / 2;
                        if (abs(mouseX - midX) < 15 && abs(mouseY - midY) < 15) {
                            selectedElement = e;
                            break;
                        }
                    }
                }

                if (selectedElement != nullptr) {
                    if (mouseX >= deleteButton.x && mouseX <= deleteButton.x + deleteButton.w &&
                        mouseY >= deleteButton.y && mouseY <= deleteButton.y + deleteButton.h) {
                        elements.erase(remove(elements.begin(), elements.end(), selectedElement), elements.end());
                        delete selectedElement;
                        selectedElement = nullptr;
                    } else if (mouseX >= valueButton.x && mouseX <= valueButton.x + valueButton.w &&
                               mouseY >= valueButton.y && mouseY <= valueButton.y + valueButton.h) {
                        valueText = "";
                        valueInputMode = true;
                    }

                }
            }
            else if (event.type == SDL_TEXTINPUT) {
                if (typingMode) typingText += event.text.text;
                else if (addressInputMode) addressText += event.text.text;
                else if (valueInputMode) valueText += event.text.text;
            }
            else if (event.type == SDL_KEYDOWN) {
                if (typingMode) {
                    if (event.key.keysym.sym == SDLK_BACKSPACE && !typingText.empty())
                        typingText.pop_back();
                    else if (event.key.keysym.sym == SDLK_RETURN) {
                        if (nodeToName && !typingText.empty())
                            nodeToName->name = typingText;
                        typingMode = false;
                        nodeToName = nullptr;
                    }
                } else if (addressInputMode) {
                    if (event.key.keysym.sym == SDLK_BACKSPACE && !addressText.empty())
                        addressText.pop_back();
                    else if (event.key.keysym.sym == SDLK_RETURN) {
                        load_elements_from_file(addressText, elements, nodes);
                        addressInputMode = false;
                    }
                } else if (valueInputMode) {
                    if (event.key.keysym.sym == SDLK_BACKSPACE && !valueText.empty())
                        valueText.pop_back();
                    else if (event.key.keysym.sym == SDLK_RETURN) {
                        if (selectedElement && !valueText.empty()) {
                            try {
                                int val = stoi(valueText);
                                selectedElement->value = val;
                            } catch (...) {
                                cerr << "Invalid value input!" << endl;
                            }
                        }
                        valueInputMode = false;
                    }
                }
            }


            else if (event.type == SDL_TEXTINPUT) {
                if (typingMode) typingText += event.text.text;
                else if (addressInputMode) addressText += event.text.text;
            }
            else if (event.type == SDL_KEYDOWN) {
                if (typingMode) {
                    if (event.key.keysym.sym == SDLK_BACKSPACE && !typingText.empty()) {
                        typingText.pop_back();
                    } else if (event.key.keysym.sym == SDLK_RETURN) {
                        if (nodeToName && !typingText.empty()) {
                            nodeToName->name = typingText;
                        }
                        typingMode = false;
                        nodeToName = nullptr;
                    }
                } else if (addressInputMode) {
                    if (event.key.keysym.sym == SDLK_BACKSPACE && !addressText.empty()) {
                        addressText.pop_back();
                    } else if (event.key.keysym.sym == SDLK_RETURN) {
                        load_elements_from_file(addressText, elements, nodes);
                        addressInputMode = false;
                    }
                }
            }

            else if (event.type == SDL_TEXTINPUT && typingMode) {
                typingText += event.text.text;
            } else if (event.type == SDL_KEYDOWN && typingMode) {
                if (event.key.keysym.sym == SDLK_BACKSPACE && !typingText.empty()) {
                    typingText.pop_back();
                } else if (event.key.keysym.sym == SDLK_RETURN) {
                    if (nodeToName) {
                        bool duplicate = false;
                        for (int i = 0; i < 7; ++i)
                            for (int j = 0; j < 10; ++j)
                                if (&nodes[i][j] != nodeToName && nodes[i][j].name == typingText)
                                    duplicate = true;

                        if (!duplicate && !typingText.empty())
                            nodeToName->name = typingText;
                    }
                    typingMode = false;
                    nodeToName = nullptr;
                }

            }



            else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
                dragging = false;
                draggedNode = nullptr;
                nameInputPending = false;
                groundTogglePending = false;
            } else if (event.type == SDL_MOUSEMOTION && dragging && draggedNode != nullptr) {
                SDL_GetMouseState(&mouseX, &mouseY);
                draggedNode->x = mouseX;
                draggedNode->y = mouseY;
            }
        }

        SDL_SetRenderDrawColor(renderer, 170, 170, 170, 255);
        SDL_RenderClear(renderer);


        SDL_SetRenderDrawColor(renderer, 50, 100, 200, 255);
        SDL_RenderFillRect(renderer, &getFileButton);
        SDL_Color white = {255, 255, 255};
        SDL_Surface* btnSurface = TTF_RenderText_Solid(font, "Get File Address", white);
        SDL_Texture* btnTexture = SDL_CreateTextureFromSurface(renderer, btnSurface);
        SDL_Rect btnTextRect = {getFileButton.x + 10, getFileButton.y + 10, btnSurface->w, btnSurface->h};
        SDL_RenderCopy(renderer, btnTexture, NULL, &btnTextRect);
        SDL_FreeSurface(btnSurface);
        SDL_DestroyTexture(btnTexture);


        SDL_SetRenderDrawColor(renderer, 100, 200, 200, 255);
        SDL_RenderFillRect(renderer, &solveButton);
        SDL_Surface* solveSurf = TTF_RenderText_Solid(font, "Solve", white);
        SDL_Texture* solveTex = SDL_CreateTextureFromSurface(renderer, solveSurf);
        SDL_Rect solveTextRect = {solveButton.x + 10, solveButton.y + 10, solveSurf->w, solveSurf->h};
        SDL_RenderCopy(renderer, solveTex, NULL, &solveTextRect);
        SDL_FreeSurface(solveSurf);
        SDL_DestroyTexture(solveTex);

        if(number_simple&&number_vahed&&number_nemad_elmi){

            SDL_SetRenderDrawColor(renderer, 200, 0, 200, 255);
            SDL_RenderFillRect(renderer, &simpleButton);
            SDL_Surface* s_Surf = TTF_RenderText_Solid(font, "simple", white);
            SDL_Texture* s_Tex = SDL_CreateTextureFromSurface(renderer, s_Surf);
            SDL_Rect s_Rect = {simpleButton.x + 10, simpleButton.y + 10, solveSurf->w, solveSurf->h};
            SDL_RenderCopy(renderer, s_Tex, NULL, &s_Rect);
            SDL_FreeSurface(s_Surf);
            SDL_DestroyTexture(s_Tex);


            SDL_SetRenderDrawColor(renderer, 200, 0, 0, 255);
            SDL_RenderFillRect(renderer, &nemad_elmiButton);
            SDL_Surface* n_Surf = TTF_RenderText_Solid(font, "nemad_elmi", white);
            SDL_Texture* n_Tex = SDL_CreateTextureFromSurface(renderer, n_Surf);
            SDL_Rect n_Rect = {nemad_elmiButton.x + 10, nemad_elmiButton.y + 10, n_Surf->w, n_Surf->h};
            SDL_RenderCopy(renderer, n_Tex, NULL, &n_Rect);
            SDL_FreeSurface(n_Surf);
            SDL_DestroyTexture(n_Tex);


            SDL_SetRenderDrawColor(renderer, 0, 200, 0, 255);
            SDL_RenderFillRect(renderer, &vahedButton);
            SDL_Surface* v_Surf = TTF_RenderText_Solid(font, "vahed", white);
            SDL_Texture* v_Tex = SDL_CreateTextureFromSurface(renderer, v_Surf);
            SDL_Rect v_Rect = {vahedButton.x + 10, vahedButton.y + 10, v_Surf->w, v_Surf->h};
            SDL_RenderCopy(renderer, v_Tex, NULL, &v_Rect);
            SDL_FreeSurface(v_Surf);
            SDL_DestroyTexture(v_Tex);
        }



        for (Element* e : elements) {
            draw_element(renderer, font, e);
        }

        if (addressInputMode) {
            SDL_Color inputColor = {255, 255, 0};
            SDL_Surface* inputSurface = TTF_RenderText_Solid(font, ("Address: " + addressText).c_str(), inputColor);
            SDL_Texture* inputTexture = SDL_CreateTextureFromSurface(renderer, inputSurface);
            SDL_Rect inputRect = {20, 50, inputSurface->w, inputSurface->h};
            SDL_RenderCopy(renderer, inputTexture, NULL, &inputRect);
            SDL_FreeSurface(inputSurface);
            SDL_DestroyTexture(inputTexture);
        }


        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 10; ++j) {
                SDL_Rect r = {nodes[i][j].x - 10, nodes[i][j].y - 10, 20, 20};
                SDL_RenderFillRect(renderer, &r);
                SDL_Color color = {255, 255, 255};
                string label = nodes[i][j].name + " (" + to_string(nodes[i][j].number) + ")";
                if (nodes[i][j].isGround) label += " GND";
                SDL_Surface* surface = TTF_RenderText_Solid(font, label.c_str(), color);
                SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
                SDL_Rect tr = {nodes[i][j].x - surface->w / 2, nodes[i][j].y - surface->h / 2, surface->w, surface->h};
                SDL_RenderCopy(renderer, texture, NULL, &tr);
                SDL_FreeSurface(surface);
                SDL_DestroyTexture(texture);
            }
        }

        if (selectedNode != nullptr) {

            if (selectedNode != nullptr &&
                mouseX >= groundButton.x && mouseX <= groundButton.x + groundButton.w &&
                mouseY >= groundButton.y && mouseY <= groundButton.y + groundButton.h) {

                if (!groundTogglePending) {
                    groundTogglePending = true;

                    try {
                        if (selectedNode->isGround) {

                            selectedNode->isGround = false;
                        } else {

                            for (int i = 0; i < 7; ++i) {
                                for (int j = 0; j < 10; ++j) {
                                    if (nodes[i][j].isGround) {
                                        throw GroundAlreadySetException();
                                    }
                                }
                            }


                            selectedNode->isGround = true;
                        }

                    } catch (const GroundAlreadySetException& e) {
                        cerr << e.what() << endl;
                    }

                    selectedNode = nullptr;
                }
            }

            else if (selectedNode != nullptr &&
                     mouseX >= nameButton.x && mouseX <= nameButton.x + nameButton.w &&
                     mouseY >= nameButton.y && mouseY <= nameButton.y + nameButton.h) {
                typingText = "";
                typingMode = true;
            }


        }

        if (selectedNode != nullptr) {
            SDL_Color white = {255, 255, 255};

            SDL_SetRenderDrawColor(renderer, 200, 150, 0, 255);
            SDL_RenderFillRect(renderer, &groundButton);
            SDL_Surface* gSurf = TTF_RenderText_Solid(font, "Ground", white);
            SDL_Texture* gTex = SDL_CreateTextureFromSurface(renderer, gSurf);
            SDL_Rect gTextRect = {groundButton.x + 10, groundButton.y + 10, gSurf->w, gSurf->h};
            SDL_RenderCopy(renderer, gTex, NULL, &gTextRect);
            SDL_FreeSurface(gSurf);
            SDL_DestroyTexture(gTex);

            SDL_SetRenderDrawColor(renderer, 100, 100, 255, 255);
            SDL_RenderFillRect(renderer, &nameButton);
            SDL_Surface* nSurf = TTF_RenderText_Solid(font, "Name", white);
            SDL_Texture* nTex = SDL_CreateTextureFromSurface(renderer, nSurf);
            SDL_Rect nTextRect = {nameButton.x + 10, nameButton.y + 10, nSurf->w, nSurf->h};
            SDL_RenderCopy(renderer, nTex, NULL, &nTextRect);
            SDL_FreeSurface(nSurf);
            SDL_DestroyTexture(nTex);
        }



        if (selectedElement != nullptr) {
            SDL_Color white = {255, 255, 255};

            SDL_SetRenderDrawColor(renderer, 200, 50, 50, 255);
            SDL_RenderFillRect(renderer, &deleteButton);
            SDL_Surface* delSurf = TTF_RenderText_Solid(font, "Delete", white);
            SDL_Texture* delTex = SDL_CreateTextureFromSurface(renderer, delSurf);
            SDL_Rect delTextRect = {deleteButton.x + 10, deleteButton.y + 10, delSurf->w, delSurf->h};
            SDL_RenderCopy(renderer, delTex, NULL, &delTextRect);
            SDL_FreeSurface(delSurf);
            SDL_DestroyTexture(delTex);

            SDL_SetRenderDrawColor(renderer, 50, 200, 50, 255);
            SDL_RenderFillRect(renderer, &valueButton);
            SDL_Surface* valSurf = TTF_RenderText_Solid(font, "Value", white);
            SDL_Texture* valTex = SDL_CreateTextureFromSurface(renderer, valSurf);
            SDL_Rect valTextRect = {valueButton.x + 10, valueButton.y + 10, valSurf->w, valSurf->h};
            SDL_RenderCopy(renderer, valTex, NULL, &valTextRect);
            SDL_FreeSurface(valSurf);
            SDL_DestroyTexture(valTex);
        }

        if (typingMode) {
            SDL_Color inputColor = {255, 255, 255};
            SDL_Surface* inputSurface = TTF_RenderText_Solid(font, ("Name: " + typingText).c_str(), inputColor);
            SDL_Texture* inputTexture = SDL_CreateTextureFromSurface(renderer, inputSurface);
            SDL_Rect inputRect = {20, 20, inputSurface->w, inputSurface->h};
            SDL_RenderCopy(renderer, inputTexture, NULL, &inputRect);
            SDL_FreeSurface(inputSurface);
            SDL_DestroyTexture(inputTexture);
        }

        if (valueInputMode) {
            SDL_Color inputColor = {255, 150, 150};
            SDL_Surface* inputSurface = TTF_RenderText_Solid(font, ("Value: " + valueText).c_str(), inputColor);
            SDL_Texture* inputTexture = SDL_CreateTextureFromSurface(renderer, inputSurface);
            SDL_Rect inputRect = {20, 80, inputSurface->w, inputSurface->h};
            SDL_RenderCopy(renderer, inputTexture, NULL, &inputRect);
            SDL_FreeSurface(inputSurface);
            SDL_DestroyTexture(inputTexture);
        }




        SDL_RenderPresent(renderer);
    }

    for (auto e : elements) delete e;
    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_StopTextInput();
    TTF_Quit();
    SDL_Quit();
    return 0;
}


void draw_element(SDL_Renderer* renderer, TTF_Font* font, Element* e) {
    if (!e || !e->from || !e->to) return;

    int x1 = e->from->x, y1 = e->from->y;
    int x2 = e->to->x, y2 = e->to->y;
    int midX = (x1 + x2) / 2;
    int midY = (y1 + y2) / 2;

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

    if (e->type == "resistor") {
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

        int zigzagStart = midX - 15;
        int step = 5;
        int zigzagLength = 30;
        for (int i = 0; i <= zigzagLength; i += step) {
            int x = zigzagStart + i;
            int y = (i / step) % 2 == 0 ? midY - 6 : midY + 6;
            int xNext = x + step;
            int yNext = ((i / step) + 1) % 2 == 0 ? midY - 6 : midY + 6;
            if (i + step <= zigzagLength)
                SDL_RenderDrawLine(renderer, x, y, xNext, yNext);
        }
    } else if (e->type == "capacitor") {
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

        SDL_RenderDrawLine(renderer, midX - 5, midY - 10, midX - 5, midY + 10);
        SDL_RenderDrawLine(renderer, midX + 5, midY - 10, midX + 5, midY + 10);
    } else if (e->type == "inductor") {
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

        int totalCircles = 4;
        int spacing = 12;
        int radius = 6;
        int startX = midX - (totalCircles * spacing) / 2 + spacing / 2;

        for (int i = 0; i < totalCircles; ++i) {
            int cx = startX + i * spacing;
            for (int angle = 0; angle < 360; angle += 10) {
                int x = cx + radius * cos(angle * M_PI / 180);
                int y = midY + radius * sin(angle * M_PI / 180);
                int x2 = cx + radius * cos((angle + 10) * M_PI / 180);
                int y2 = midY + radius * sin((angle + 10) * M_PI / 180);
                SDL_RenderDrawLine(renderer, x, y, x2, y2);
            }
        }
    } else if (e->type == "voltageSource") {
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

        SDL_Rect circle = {midX - 10, midY - 10, 20, 20};
        SDL_RenderDrawRect(renderer, &circle);
        SDL_RenderDrawLine(renderer, midX, midY - 6, midX, midY + 6);
        SDL_RenderDrawLine(renderer, midX - 5, midY, midX + 5, midY);
        SDL_RenderDrawLine(renderer, midX - 5, midY + 6, midX + 5, midY + 6);
        SDL_RenderDrawLine(renderer, midX - 5, midY + 6, midX + 5, midY + 6);
        circle = {midX - 10, midY - 10, 20, 20};
        SDL_RenderDrawRect(renderer, &circle);
        SDL_RenderDrawLine(renderer, midX, midY - 6, midX, midY + 6);
        SDL_RenderDrawLine(renderer, midX - 5, midY, midX + 5, midY);
    } else if (e->type == "currentSource") {
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

        SDL_Rect circle = {midX - 10, midY - 10, 20, 20};
        SDL_RenderDrawRect(renderer, &circle);
        SDL_RenderDrawLine(renderer, midX, midY + 5, midX, midY - 5);
        SDL_RenderDrawLine(renderer, midX, midY - 5, midX - 5, midY);
        SDL_RenderDrawLine(renderer, midX, midY - 5, midX + 5, midY);
    }

    string label = e->name + " (" + to_string(e->value) + ")";

    SDL_Color red = {255, 0, 0};
    SDL_Surface* surf = TTF_RenderText_Solid(font, label.c_str(), red);
    SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
    SDL_Rect txtRect = {midX - surf->w / 2, midY - 25, surf->w, surf->h};
    SDL_RenderCopy(renderer, tex, NULL, &txtRect);
    SDL_FreeSurface(surf);
    SDL_DestroyTexture(tex);

}


void load_elements_from_file(const string& filename, vector<Element*>& elements, Node nodes[7][10]) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    string line;
    smatch matcher;
    regex pattern[5];
    pattern[0] = regex(R"(^add\s+resistor\s+(\w+)\s+(\d+)\s+(\d+)\s+(\d+)$)");
    pattern[1] = regex(R"(^add\s+capacitor\s+(\w+)\s+(\d+)\s+(\d+)\s+(\d+)$)");
    pattern[2] = regex(R"(^add\s+inductor\s+(\w+)\s+(\d+)\s+(\d+)\s+(\d+)$)");
    pattern[3] = regex(R"(^add\s+voltageSource\s+(\w+)\s+(\d+)\s+(\d+)\s+(\d+)$)");
    pattern[4] = regex(R"(^add\s+currentSource\s+(\w+)\s+(\d+)\s+(\d+)\s+(\d+)$)");

    while (getline(file, line)) {
        for (int i = 0; i < 5; ++i) {
            if (regex_match(line, matcher, pattern[i])) {
                string name = matcher[1];
                int from_index = stoi(matcher[2]);
                int to_index = stoi(matcher[3]);
                int value = stoi(matcher[4]);

                if (from_index >= 70 || to_index >= 70) {
                    cerr << "Invalid node index in: " << line << endl;
                    break;
                }

                Node* from = &nodes[from_index / 10][from_index % 10];
                Node* to = &nodes[to_index / 10][to_index % 10];

                from->connect = true;
                to->connect = true;

                switch (i) {
                    case 0: elements.push_back(new Resistor(name, value, from, to)); break;
                    case 1: elements.push_back(new Capacitor(name, value, from, to)); break;
                    case 2: elements.push_back(new Inductor(name, value, from, to)); break;
                    case 3: elements.push_back(new VoltageSource(name, value, from, to)); break;
                    case 4: elements.push_back(new CurrentSource(name, value, from, to)); break;
                }
                break;
            }
        }
    }
    file.close();
}


string toEngineering(double value) {
    const char* prefixes[] = {"p", "n", "u", "m", "", "k", "M", "G"};
    int index = 4;

    if (value == 0)
        return "0";

    while (value >= 1000.0 && index < 7) {
        value /= 1000.0;
        index++;
    }
    while (value < 1.0 && index > 0) {
        value *= 1000.0;
        index--;
    }

    ostringstream oss;
    oss << fixed << setprecision(2) << value << prefixes[index];
    return oss.str();
}
