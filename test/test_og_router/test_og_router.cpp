#include <algorithm>
#include <unistd.h>
// SPDX-License-Identifier: MIT
//
// Host-side dry run of the OG (rev 2) router, src/routing/NetsToChipConnections_OG.cpp.
//
// Loads nets exactly as NetManager leaves them, runs bridgesToPaths(), then
// SIMULATES THE CH446Q CROSSBAR the way CH446Q.cpp sendPath() drives it (every
// chip[h]/x[h]/y[h] with x,y >= 0 closes one crosspoint) on a model of the rev 2
// wiring (board_og.cpp xMap/yMap: BB chip Y0 = chip L Y[chip], X lanes between
// chips, L's X pins = SF nodes/corners). It then checks, per net, that every
// member node is electrically reachable from the first one and that no net
// touches another net or an unrelated SF node. The path table is NOT trusted;
// only the closed crosspoints are.
//
// Cases 1-3 are the failures measured on a real rev 2 board with backport
// 1.7.11.1 (2026-09-11): corner rows (1/30/31/60) straight to an SF node, two
// corners at once, and a >8-row GND net beside a 3V3 row. "rand" is a
// randomized sweep that reports how many trials leave a node unrouted and how
// many produce a SHORT (the number that must stay 0).
//
// Build & run (no hardware, no PlatformIO):   test/test_og_router/run.sh
//   ./test_og_router            the fixed cases (exit 1 on any failure)
//   ./test_og_router v          same, with the router's own debug trace
//   ./test_og_router rand SEED N [x [y TRIAL]]   random sweep (x: list shorting
//                               trials, y TRIAL: replay one trial verbosely)
#include <Arduino.h>
#include <map>
#include <set>
#include <vector>
#include <string>
#include "States.h"
#include "NetsToChipConnections.h"
#include "NetManager.h"
#include "FakeGpio.h"
#include "Peripherals.h"
#include "Graphics.h"
#include "boards/board.h"

Stream Serial; Stream Serial1; Stream Jerial;
JumperlessState globalState;
JumperlessConfig jumperlessConfig;
FakeGpioOutput fakeGpioOutputs[MAX_FAKE_GP_OUT];
FakeGpioInput fakeGpioInputs[MAX_FAKE_GP_IN];
int fakeGpioInputAdcChannel = -1;
int gpioNet[10]; int gpioReading[10]; int gpioDef[10][3]; int showADCreadings[8]; uint32_t gpioReadingColors[10];
int newBridgeLength = 0; int numberOfShownNets = 0;
#include "nano_init.inc"
void initNets(void) {}
bool infraIsBridge(int, int) { return false; }
void assignTermColor(int) {}
void printBridgeArray(Stream*) {}
static std::string nodeName(int n) {
  switch (n) {
    case GND: return "GND"; case SUPPLY_3V3: return "3V3"; case SUPPLY_5V: return "5V";
    case DAC0: return "DAC0"; case DAC1: return "DAC1"; case ADC0: return "ADC0"; case ADC1: return "ADC1";
    case ADC2: return "ADC2"; case ADC3: return "ADC3"; case RP_GPIO_0: return "GPIO_0";
    case RP_UART_TX: return "UART_TX"; case RP_UART_RX: return "UART_RX";
    case ISENSE_PLUS: return "I+"; case ISENSE_MINUS: return "I-";
    case NANO_RESET: return "NRST"; case NANO_AREF: return "AREF";
  }
  if (n >= 1 && n <= 60) return std::to_string(n);
  if (n >= NANO_D0 && n <= NANO_D13) return "D" + std::to_string(n - NANO_D0);
  if (n >= NANO_A0 && n <= NANO_A7) return "A" + std::to_string(n - NANO_A0);
  return "n" + std::to_string(n);
}
int printNodeOrName(int node, int, int, Stream* s) { return s->print(nodeName(node)); }
const char* definesToChar(int n, int) { static std::string s; s = nodeName(n); return s.c_str(); }

// ---------- physical crossbar model (OG rev 2) ----------
static const board::BoardTopology& B = board::ogBoardTopology;
struct UF { std::map<std::string,std::string> p; std::string f(std::string a){ if(!p.count(a)) p[a]=a; while(p[a]!=a){ p[a]=p[p[a]]; a=p[a]; } return a; } void u(std::string a,std::string b){ p[f(a)]=f(b);} };
static std::string laneName(int c, int x) {
  int t = B.xMap[c][x];
  if (c < 8) {
    if (t < 8) { int k = 0; for (int i = 0; i < x; i++) if (B.xMap[c][i] == t) k++;
      int lo = c < t ? c : t, hi = c < t ? t : c; return "lane_bb" + std::to_string(lo) + "_" + std::to_string(hi) + "_" + std::to_string(k); }
    return "lane_sf" + std::to_string(t) + "_bb" + std::to_string(c);
  }
  return "node_" + nodeName(t);
}
static std::string yName_(int c, int y) {
  int t = B.yMap[c][y];
  if (c < 8) { if (y == 0) return "lane_L_bb" + std::to_string(c); return "node_" + nodeName(t); }
  if (c == CHIP_L) return "lane_L_bb" + std::to_string(t);
  return "lane_sf" + std::to_string(c) + "_bb" + std::to_string(t);
}
struct NetDef { int number; std::vector<int> nodes; std::vector<std::pair<int,int>> bridges; };
static bool runCase(const char* title, std::vector<NetDef> defs, bool verbose) {
  printf("\n=== %s ===\n", title);
  memset(&globalState, 0, sizeof(globalState));
  for (int i = 0; i < 12; i++) for (int j = 0; j < 16; j++) globalState.connections.chipStates[i].xMap[j] = B.xMap[i][j];
  for (int i = 0; i < 12; i++) for (int j = 0; j < 8; j++) globalState.connections.chipStates[i].yMap[j] = B.yMap[i][j];
  for (int i = 0; i < 12; i++) { globalState.connections.chipStates[i].chipNumber = i; globalState.connections.chipStates[i].chipChar = 'A' + i; }
  clearAllNTCC(); // firmware order: clear, then NetManager fills nets, then bridgesToPaths
  // base nets 1..5 like initNets: GND, TOP_RAIL, BOTTOM_RAIL, DAC0, DAC1
  const int baseNode[6] = {0, GND, TOP_RAIL, BOTTOM_RAIL, DAC0, DAC1};
  for (int i = 1; i <= 5; i++) { globalState.connections.nets[i].number = i; globalState.connections.nets[i].nodes[0] = baseNode[i]; globalState.connections.nets[i].specialFunction = baseNode[i]; }
  int nb = 0;
  for (auto& d : defs) {
    netStruct& n = globalState.connections.nets[d.number]; n.number = d.number; n.specialFunction = -1;
    // Mirror NetManager: nodes[]/bridges[] are MAX_NODES deep per net; anything
    // past that is dropped (addNodeToNet/addBridgeToNet) and is NOT expected to
    // route. The test reports the drop so the cap stays visible.
    if (d.bridges.size() > (size_t)MAX_NODES) { printf("  NOTE net %d: %zu bridges, only %d fit (MAX_NODES) - extras dropped like NetManager does\n", d.number, d.bridges.size(), MAX_NODES); d.bridges.resize(MAX_NODES); }
    if (d.nodes.size() > (size_t)MAX_NODES) {
      // nodes[] overflows one entry earlier than bridges[] (the first node has no
      // bridge). Routing works from the bridge table, so the expectation is the
      // set of nodes the TRACKED bridges mention; nodes[] itself is just capped.
      printf("  NOTE net %d: %zu nodes, only %d fit in nodes[] (display list) - routing follows the bridges\n", d.number, d.nodes.size(), MAX_NODES);
      std::vector<int> keep; for (int n : d.nodes) { bool used = false; for (auto& b : d.bridges) if (b.first == n || b.second == n) used = true; if (used) keep.push_back(n); }
      d.nodes = keep;
    }
    for (size_t i = 0; i < d.nodes.size(); i++) n.nodes[i] = d.nodes[i];
    for (size_t i = 0; i < d.bridges.size(); i++) { n.bridges[i][0] = d.bridges[i].first; n.bridges[i][1] = d.bridges[i].second;
      globalState.connections.bridges[nb][0] = d.bridges[i].first; globalState.connections.bridges[nb][1] = d.bridges[i].second; nb++; }
  }
  globalState.connections.numBridges = nb;
  int maxNet = 5; for (auto& d : defs) if (d.number > maxNet) maxNet = d.number;
  for (int i = 6; i <= maxNet; i++) if (globalState.connections.nets[i].number == 0) globalState.connections.nets[i].number = i;
  if (verbose) { for (int j=1;j<8;j++){ printf("  net[%d] number=%d bridges0=%d,%d\n", j, globalState.connections.nets[j].number, globalState.connections.nets[j].bridges[0][0], globalState.connections.nets[j].bridges[0][1]); } }
  bridgesToPaths();
  if (verbose) printf("  numberOfPaths=%d\n", (int)numberOfPaths);
  for (int c = 0; c < 12; c++) for (int j = 0; j < 16; j++) if (globalState.connections.chipStates[c].xMap[j] != B.xMap[c][j]) printf("  CORRUPT: chip %c xMap[%d] = %d (expected %d)\n", 'A'+c, j, globalState.connections.chipStates[c].xMap[j], B.xMap[c][j]);
  for (int c = 0; c < 12; c++) for (int j = 0; j < 8; j++) if (globalState.connections.chipStates[c].yMap[j] != B.yMap[c][j]) printf("  CORRUPT: chip %c yMap[%d] = %d (expected %d)\n", 'A'+c, j, globalState.connections.chipStates[c].yMap[j], B.yMap[c][j]);
  if (verbose) { printPathsCompact(); printChipStatus(); }
  // simulate what sendPath() closes
  UF uf; std::map<std::string,std::set<int>> laneNets;
  for (int i = 0; i < numberOfPaths; i++) {
    auto& p = globalState.connections.paths[i]; if (p.skip) continue;
    for (int h = 0; h < 4; h++) { if (p.chip[h] == -1 || p.x[h] < 0 || p.y[h] < 0) continue;
      std::string a = laneName(p.chip[h], p.x[h]), b = yName_(p.chip[h], p.y[h]); uf.u(a, b); }
  }
  bool ok = true;
  for (auto& d : defs) {
    std::string root = uf.f("node_" + nodeName(d.nodes[0]));
    for (size_t i = 1; i < d.nodes.size(); i++) { std::string r = uf.f("node_" + nodeName(d.nodes[i]));
      if (r != root) { printf("  FAIL net %d: %s is NOT connected to %s\n", d.number, nodeName(d.nodes[i]).c_str(), nodeName(d.nodes[0]).c_str()); ok = false; } }
  }
  for (size_t a = 0; a < defs.size(); a++) for (size_t b2 = a + 1; b2 < defs.size(); b2++) {
    if (uf.f("node_" + nodeName(defs[a].nodes[0])) == uf.f("node_" + nodeName(defs[b2].nodes[0]))) { printf("  FAIL: net %d SHORTED to net %d\n", defs[a].number, defs[b2].number); ok = false; } }
  // any user net shorted to an unrelated SF node?
  const int sfs[] = {GND, SUPPLY_3V3, SUPPLY_5V, DAC0, DAC1, ADC0, ADC1, ADC2, ADC3, RP_GPIO_0, RP_UART_TX, RP_UART_RX, ISENSE_PLUS, ISENSE_MINUS, NANO_RESET, NANO_AREF};
  for (auto& d : defs) for (int s : sfs) { bool member = false; for (int n : d.nodes) if (n == s) member = true; if (member) continue;
    if (uf.p.count("node_" + nodeName(s)) && uf.f("node_" + nodeName(s)) == uf.f("node_" + nodeName(d.nodes[0]))) { printf("  FAIL: net %d touches unrelated node %s\n", d.number, nodeName(s).c_str()); ok = false; } }
  printf("  paths=%d  %s\n", (int)numberOfPaths, ok ? "PASS" : "FAIL");
  return ok;
}
static int quiet = 0;
static bool runCaseQ(std::vector<NetDef> defs, int& shorts, int& unrouted) {
  // same as runCase but silent; returns ok and tallies
  int f = 0; (void)f;
  fflush(stdout); int saved = dup(1); FILE* tmp = tmpfile(); dup2(fileno(tmp), 1);
  bool ok = runCase("rand", defs, false);
  fflush(stdout); dup2(saved, 1); close(saved);
  rewind(tmp); char line[512]; while (fgets(line, sizeof line, tmp)) { if (strstr(line, "SHORTED") || strstr(line, "touches")) shorts++; else if (strstr(line, "NOT connected")) unrouted++; }
  fclose(tmp);
  return ok;
}
int main(int argc, char** argv) {
  if (argc > 2 && std::string(argv[1]) == "rand") {
    unsigned seed = atoi(argv[2]); int n = argc > 3 ? atoi(argv[3]) : 200; srand(seed);
    debugNTCC = false; debugNTCC2 = false;
    int fails = 0; int total = 0; int shortTrials = 0;
    const int sfPool[] = {GND, SUPPLY_3V3, SUPPLY_5V, DAC0, DAC1, ADC0, ADC1, ADC2, ADC3, RP_GPIO_0, RP_UART_TX, RP_UART_RX};
    for (int t = 0; t < n; t++) {
      std::vector<NetDef> defs; std::set<int> used; int netNo = 6; int gndUsed = 0;
      int nets = 1 + rand() % 4;
      for (int k = 0; k < nets; k++) {
        NetDef d; int sf = -1; if (rand() % 3) { sf = sfPool[rand() % 12]; if (used.count(sf)) sf = -1; }
        if (sf == GND && gndUsed) sf = -1;
        if (sf != -1) { used.insert(sf); d.nodes.push_back(sf); }
        int rows = 1 + rand() % 6; if (sf == -1 && rows < 2) rows = 2;
        for (int r = 0; r < rows; r++) { int row; int tries = 0; do { row = 1 + rand() % 60; } while (used.count(row) && ++tries < 100); if (used.count(row)) continue; used.insert(row); d.nodes.push_back(row); }
        if ((int)d.nodes.size() < 2) continue;
        for (size_t i = 1; i < d.nodes.size(); i++) d.bridges.push_back({d.nodes[rand() % i], d.nodes[i]});
        if (sf == GND) { d.number = 1; gndUsed = 1; } else d.number = netNo++;
        defs.push_back(d);
      }
      if (defs.empty()) continue;
      if (argc > 6 && atoi(argv[6]) != t) continue;
      if (argc > 6) { debugNTCC2 = true; runCase("replay", defs, true); return 0; }
      total++;
      int sh = 0, un = 0;
      if (!runCaseQ(defs, sh, un)) { fails++; if (sh) shortTrials++; if (argc > 4 && (sh || argc > 5)) { printf("seed %u trial %d FAILED:", seed, t); for (auto& d : defs) { printf(" net%d{", d.number); for (int x : d.nodes) printf("%s ", nodeName(x).c_str()); printf("}"); } printf("\n"); } }
    }
    printf("random sweep seed=%u: %d/%d trials failed, %d with SHORTS\n", seed, fails, total, shortTrials);
    return 0;
  }
  bool verbose = argc > 1; debugNTCC = verbose; debugNTCC2 = verbose;
  int fails = 0;
  // Case 1 (bug 1): a corner row straight to an SF node.
  fails += !runCase("1: 3V3-1 (corner direct to SF)", {{6, {SUPPLY_3V3, 1}, {{SUPPLY_3V3, 1}}}}, verbose);
  fails += !runCase("1b: GND-30", {{1, {GND, 30}, {{GND, 30}}}}, verbose);
  fails += !runCase("1c: ADC0-60", {{6, {ADC0, 60}, {{ADC0, 60}}}}, verbose);
  // Case 2 (bug 2): two corners at once.
  fails += !runCase("2: {3V3,5,1} + {GND,28,30}", {{6, {SUPPLY_3V3, 5, 1}, {{SUPPLY_3V3, 5}, {5, 1}}}, {1, {GND, 28, 30}, {{GND, 28}, {28, 30}}}}, verbose);
  fails += !runCase("2r: {GND,28,30} + {3V3,5,1} (reordered)", {{1, {GND, 28, 30}, {{GND, 28}, {28, 30}}}, {6, {SUPPLY_3V3, 5, 1}, {{SUPPLY_3V3, 5}, {5, 1}}}}, verbose);
  fails += !runCase("2b: 3V3-1 + GND-30 (direct corners)", {{6, {SUPPLY_3V3, 1}, {{SUPPLY_3V3, 1}}}, {1, {GND, 30}, {{GND, 30}}}}, verbose);
  fails += !runCase("2c: all four corners", {{6, {SUPPLY_3V3, 1}, {{SUPPLY_3V3, 1}}}, {1, {GND, 30}, {{GND, 30}}}, {7, {ADC0, 31}, {{ADC0, 31}}}, {8, {DAC0, 60}, {{DAC0, 60}}}}, verbose);
  // Case 3 (bug 3): a big GND net next to 3V3 on the same chip.
  { NetDef g{1, {GND}, {}}; for (int r = 4; r <= 11; r++) { g.nodes.push_back(r); g.bridges.push_back({GND, r}); }
    fails += !runCase("3: 3V3-3 + GND-4..11", {{6, {SUPPLY_3V3, 3}, {{SUPPLY_3V3, 3}}}, g}, verbose); }
  { NetDef g{1, {GND}, {}}; for (int r = 16; r <= 30; r++) { g.nodes.push_back(r); g.bridges.push_back({GND, r}); }
    fails += !runCase("3b: 3V3-3 + GND-16..30", {{6, {SUPPLY_3V3, 3}, {{SUPPLY_3V3, 3}}}, g}, verbose); }
  { NetDef g{1, {GND}, {}}; for (int r = 4; r <= 11; r++) { g.nodes.push_back(r); g.bridges.push_back({GND, r}); }
    fails += !runCase("3c: GND-4..11 alone", {g}, verbose); }
  // Bug 4 (2026-09-11, on the fixed firmware): GND on rows 1..N. N <= 20 fine,
  // N = 24 drops EVERY row, N = 60 keeps only row 24.
  // The bench repro adds the ADC0 probe bridge LAST, so it was the one the
  // per-net bridge table (MAX_NODES deep, 24 at the time) dropped: GND-1..24
  // filled it and ADC0 never routed, every row read floating. The router was
  // fine; the cap was hit one bridge early. Cases: N=24 (+ADC0 = 25 bridges,
  // the bench failure), N=MAX_NODES-1 (+ADC0 = exactly full), and N=60 (still
  // over the cap: the test expects only the tracked bridges to route, and the
  // NOTE lines make the drop visible).
  for (int n : {20, 24, MAX_NODES - 1, MAX_NODES, 60}) {
    NetDef g{1, {GND}, {}}; for (int r = 1; r <= n; r++) { g.nodes.push_back(r); g.bridges.push_back({GND, r}); }
    g.nodes.push_back(ADC0); g.bridges.push_back({ADC0, n < 12 ? n : 12});
    char t[64]; snprintf(t, sizeof t, "4: GND-1..%d + ADC0-12 probe last", n);
    fails += !runCase(t, {g}, verbose);
  }
  // sanity: the thing that works on hardware
  fails += !runCase("S: 3V3-5 + 5-1 (works on hw)", {{6, {SUPPLY_3V3, 5, 1}, {{SUPPLY_3V3, 5}, {5, 1}}}}, verbose);
  printf("\n%d failing cases\n", fails);
  return fails ? 1 : 0;
}
