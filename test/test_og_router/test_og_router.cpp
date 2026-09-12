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
#include "PathHealth.h"

Stream Serial; Stream Serial1; Stream Jerial;
JumperlessState globalState;
#ifdef NETBRIDGES_H
NetBridgePool netBridgePool;   // the firmware defines this next to globalState (States.cpp)
#else
// Building against a tree that predates routing/NetBridges.h (the digest
// baseline): the same API over the old inline per-net table.
namespace netbridges {
  inline bool append(netStruct& n, int16_t a, int16_t b) { for (int k = 0; k < MAX_NODES; k++) if (n.bridges[k][0] == 0) { n.bridges[k][0] = a; n.bridges[k][1] = b; return true; } return false; }
  inline int capacity() { return MAX_NODES; }
  inline int count(const netStruct& n) { int k = 0; while (k < MAX_NODES && n.bridges[k][0] != 0) k++; return k; }
}
#endif
JumperlessConfig jumperlessConfig;
FakeGpioOutput fakeGpioOutputs[MAX_FAKE_GP_OUT];
FakeGpioInput fakeGpioInputs[MAX_FAKE_GP_IN];
int fakeGpioInputAdcChannel = -1;
int gpioNet[10]; int gpioReading[10]; int gpioDef[10][3]; int showADCreadings[8]; uint32_t gpioReadingColors[10];
int newBridgeLength = 0; int numberOfShownNets = 0;
#include "nano_init.inc"
// The firmware's initNets() reinitialises every net and resets the per-net
// bridge pool; the harness fills the nets itself, so the stub keeps the reset.
void initNets(void) {
#ifdef NETBRIDGES_H
  netbridges::resetAll();
#endif
}
bool infraIsBridge(int, int) { return false; }
void assignTermColor(int) {}
void printBridgeArray(Stream*) {}
static bool digest = false;   // print the closed crosspoints per case/trial (compare two builds)
static std::string lastDigest; // the closed crosspoints of the last runCase
static long healthBridges = 0, healthUnrouted = 0, healthUnroutedButJoined = 0;   // PathHealth rule tallies
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
    // Mirror NetManager: nodes[] is MAX_NODES deep per net (addNodeToNet drops
    // the rest and says so); the per-net bridge list is whatever
    // netbridges::append accepts (V5 / the old tree: MAX_NODES slots; OG: the
    // shared pool, which a MAX_BRIDGES netlist cannot fill). A dropped bridge
    // is NOT expected to route; the test reports every drop so a cap stays
    // visible.
    bool nodesCapped = d.nodes.size() > (size_t)MAX_NODES;
    if (nodesCapped) printf("  NOTE net %d: %zu nodes, only %d fit in nodes[] (display list) - routing follows the bridges\n", d.number, d.nodes.size(), MAX_NODES);
    for (size_t i = 0; i < d.nodes.size() && i < (size_t)MAX_NODES; i++) n.nodes[i] = d.nodes[i];
    std::vector<std::pair<int,int>> tracked;
    for (size_t i = 0; i < d.bridges.size(); i++) {
      if (!netbridges::append(n, d.bridges[i].first, d.bridges[i].second)) {
        printf("  NOTE net %d: bridge %zu of %zu dropped - per-net bridge storage full (capacity %d) like NetManager does\n", d.number, i + 1, d.bridges.size(), netbridges::capacity());
        break;
      }
      tracked.push_back(d.bridges[i]);
      globalState.connections.bridges[nb][0] = d.bridges[i].first; globalState.connections.bridges[nb][1] = d.bridges[i].second; nb++; }
    if (tracked.size() != d.bridges.size() || nodesCapped) {
      // expectation = the nodes the TRACKED bridges mention (routing works
      // from the bridge list; nodes[] is the display list)
      d.bridges = tracked;
      std::vector<int> keep; for (int x : d.nodes) { bool used = false; for (auto& b : d.bridges) if (b.first == x || b.second == x) used = true; if (used) keep.push_back(x); }
      d.nodes = keep;
    }
  }
  globalState.connections.numBridges = nb;
  int maxNet = 5; for (auto& d : defs) if (d.number > maxNet) maxNet = d.number;
  for (int i = 6; i <= maxNet; i++) if (globalState.connections.nets[i].number == 0) globalState.connections.nets[i].number = i;
  if (verbose) { for (int j=1;j<8;j++){ printf("  net[%d] number=%d bridges=%d\n", j, globalState.connections.nets[j].number, netbridges::count(globalState.connections.nets[j])); } }
  bridgesToPaths();
  if (verbose) printf("  numberOfPaths=%d\n", (int)numberOfPaths);
  for (int c = 0; c < 12; c++) for (int j = 0; j < 16; j++) if (globalState.connections.chipStates[c].xMap[j] != B.xMap[c][j]) printf("  CORRUPT: chip %c xMap[%d] = %d (expected %d)\n", 'A'+c, j, globalState.connections.chipStates[c].xMap[j], B.xMap[c][j]);
  for (int c = 0; c < 12; c++) for (int j = 0; j < 8; j++) if (globalState.connections.chipStates[c].yMap[j] != B.yMap[c][j]) printf("  CORRUPT: chip %c yMap[%d] = %d (expected %d)\n", 'A'+c, j, globalState.connections.chipStates[c].yMap[j], B.yMap[c][j]);
  if (verbose) { printPathsCompact(); printChipStatus(); }
  // simulate what sendPath() closes
  UF uf; std::map<std::string,std::set<int>> laneNets;
  std::set<std::string> closed;   // the exact crosspoints, for the digest
  for (int i = 0; i < numberOfPaths; i++) {
    auto& p = globalState.connections.paths[i]; if (p.skip) continue;
    for (int h = 0; h < 4; h++) { if (p.chip[h] == -1 || p.x[h] < 0 || p.y[h] < 0) continue;
      std::string a = laneName(p.chip[h], p.x[h]), b = yName_(p.chip[h], p.y[h]); uf.u(a, b);
      char cp[16]; snprintf(cp, sizeof cp, "%c%d.%d", 'A' + p.chip[h], (int)p.x[h], (int)p.y[h]); closed.insert(cp); }
  }
  lastDigest.clear(); for (auto& c : closed) { lastDigest += c; lastDigest += ' '; }
  if (digest) { printf("  DIGEST paths=%d: %s\n", (int)numberOfPaths, lastDigest.c_str()); }
  bool ok = true;
  // get_netlist()'s unrouted rule (routing/PathHealth.h) against the crossbar
  // model: a bridge the rule calls CLEAN must be electrically closed - its
  // path's crosspoints connect its two nodes. (The converse is not required:
  // an unrouted bridge's nodes may still meet through other bridges of the
  // net.) Tallied per case for the report.
  for (int i = 0; i < globalState.connections.numBridges; i++) {
    int a = globalState.connections.bridges[i][0], b = globalState.connections.bridges[i][1];
    bool unrouted = pathHealthBridgeUnrouted(i, numberOfPaths) != 0;
    bool joined = uf.p.count("node_" + nodeName(a)) && uf.p.count("node_" + nodeName(b)) && uf.f("node_" + nodeName(a)) == uf.f("node_" + nodeName(b));
    healthBridges++; if (unrouted) healthUnrouted++;
    if (!unrouted && !joined) { printf("  FAIL health: bridge %s-%s is CLEAN by the rule but its nodes are not joined\n", nodeName(a).c_str(), nodeName(b).c_str()); ok = false; }
    if (unrouted && joined) healthUnroutedButJoined++;
  }
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
static bool runCaseQ(std::vector<NetDef> defs, int& shorts, int& unrouted, int trial = -1) {
  // same as runCase but silent; returns ok and tallies (DIGEST lines pass through)
  int f = 0; (void)f;
  fflush(stdout); int saved = dup(1); FILE* tmp = tmpfile(); dup2(fileno(tmp), 1);
  bool ok = runCase("rand", defs, false);
  fflush(stdout); dup2(saved, 1); close(saved);
  rewind(tmp); char line[4096]; while (fgets(line, sizeof line, tmp)) { if (strstr(line, "SHORTED") || strstr(line, "touches")) shorts++; else if (strstr(line, "NOT connected")) unrouted++; else if (digest && strstr(line, "DIGEST")) printf("trial %d %s", trial, line); }
  fclose(tmp);
  return ok;
}
int main(int argc, char** argv) {
  if (getenv("OG_ROUTER_DIGEST")) digest = true;
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
      if (!runCaseQ(defs, sh, un, t)) { fails++; if (sh) shortTrials++; if (argc > 4 && (sh || argc > 5)) { printf("seed %u trial %d FAILED:", seed, t); for (auto& d : defs) { printf(" net%d{", d.number); for (int x : d.nodes) printf("%s ", nodeName(x).c_str()); printf("}"); } printf("\n"); } }
    }
    printf("random sweep seed=%u: %d/%d trials failed, %d with SHORTS; PathHealth: %ld bridges, %ld unrouted, 0 clean-but-open\n", seed, fails, total, shortTrials, healthBridges, healthUnrouted);
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
  // Bug 4 (2026-09-11): GND on rows 1..N, then the ADC0 probe bridge added
  // LAST (the bench repro: every row read floating because ADC0 never routed).
  // The per-net bridge table was MAX_NODES=24 deep, so at N=24 the probe was
  // the 25th bridge and NetManager dropped it. The OG now files bridges in a
  // shared pool (routing/NetBridges.h) with no per-net cap, so EVERY N routes
  // the probe, up to the whole board (N=60: 61 bridges, 62 nodes - the node
  // list is 64 deep). N=40/60 also exercise a corner (31) reached through
  // lanes the net already owns (the L-hop same-net fix). If a NOTE line ever
  // appears here again, a cap is back.
  for (int n : {20, 23, 24, 40, 60}) {
    NetDef g{1, {GND}, {}}; for (int r = 1; r <= n; r++) { g.nodes.push_back(r); g.bridges.push_back({GND, r}); }
    g.nodes.push_back(ADC0); g.bridges.push_back({ADC0, n < 12 ? n : 12});
    char t[64]; snprintf(t, sizeof t, "4: GND-1..%d + ADC0-12 probe last", n);
    fails += !runCase(t, {g}, verbose);
  }
  // The pool holds 2*MAX_BRIDGES entries so a full MAX_BRIDGES netlist always
  // fits: 72 bridges spread over 6 nets, none dropped, all routed or at least
  // never shorted (the crossbar cannot route everything - the check that
  // matters is no NOTE/drop and no short). Nets: GND on 1..30 (30 bridges),
  // 3V3 on 32..49 (18), DAC0 on 50..55 (6), ADC0 on 56..60 (5), and two
  // row-only nets (7 + 6 bridges) = 72.
  {
    std::vector<NetDef> big;
    { NetDef g{1, {GND}, {}}; for (int r = 1; r <= 30; r++) { g.nodes.push_back(r); g.bridges.push_back({GND, r}); } big.push_back(g); }
    { NetDef g{6, {SUPPLY_3V3}, {}}; for (int r = 32; r <= 49; r++) { g.nodes.push_back(r); g.bridges.push_back({SUPPLY_3V3, r}); } big.push_back(g); }
    { NetDef g{7, {DAC0}, {}}; for (int r = 50; r <= 55; r++) { g.nodes.push_back(r); g.bridges.push_back({DAC0, r}); } big.push_back(g); }
    { NetDef g{8, {ADC0}, {}}; for (int r = 56; r <= 60; r++) { g.nodes.push_back(r); g.bridges.push_back({ADC0, r}); } big.push_back(g); }
    { NetDef g{9, {NANO_D0}, {}}; for (int k = 1; k <= 7; k++) { g.nodes.push_back(NANO_D0 + k); g.bridges.push_back({NANO_D0, NANO_D0 + k}); } big.push_back(g); }
    { NetDef g{10, {NANO_A0}, {}}; for (int k = 1; k <= 6; k++) { g.nodes.push_back(NANO_A0 + k); g.bridges.push_back({NANO_A0, NANO_A0 + k}); } big.push_back(g); }
    int total = 0; for (auto& d : big) total += (int)d.bridges.size();
    if (total != MAX_BRIDGES) { printf("test bug: %d bridges, wanted MAX_BRIDGES=%d\n", total, MAX_BRIDGES); fails++; }
    // this one is allowed to leave nodes unrouted (crossbar capacity), not to short or drop
    fflush(stdout); int saved = dup(1); FILE* tmp = tmpfile(); dup2(fileno(tmp), 1);
    runCase("6: MAX_BRIDGES bridges in 6 nets", big, verbose);
    fflush(stdout); dup2(saved, 1); close(saved); rewind(tmp);
    int shorts = 0, drops = 0, unrouted = 0; char line[4096];
    while (fgets(line, sizeof line, tmp)) { if (strstr(line, "SHORTED") || strstr(line, "touches")) shorts++; if (strstr(line, "NOTE")) drops++; if (strstr(line, "NOT connected")) unrouted++; }
    fclose(tmp);
    printf("\n=== 6: MAX_BRIDGES=%d bridges in 6 nets ===\n  drops=%d shorts=%d unrouted=%d  %s\n", MAX_BRIDGES, drops, shorts, unrouted, (drops || shorts) ? "FAIL" : "PASS");
    fails += (drops || shorts) ? 1 : 0;
  }
#ifdef NETBRIDGES_H
  // The OG pool itself: append / count / iteration order / clear frees /
  // detach does not / merge keeps A's bridges before B's / exhaustion is
  // reported by append returning false, never by writing out of bounds.
  {
    printf("\n=== 7: netbridges pool lifecycle ===\n");
    bool ok = true;
    netbridges::resetAll();
    netStruct a{}, b{};
    for (int i = 1; i <= 10; i++) ok &= netbridges::append(a, i, i + 1);
    for (int i = 1; i <= 5; i++) ok &= netbridges::append(b, 100 + i, 200 + i);
    ok &= netbridges::count(a) == 10 && netbridges::count(b) == 5 && netbridges::poolUsed() == 15;
    { int i = 1; for (auto it = netbridges::begin(a); it.valid(); it.next(), i++) ok &= it.node1() == i && it.node2() == i + 1; ok &= i == 11; }
    // merge the way combineNets does: append b's list to a, then free b
    for (auto it = netbridges::begin(b); it.valid(); it.next()) ok &= netbridges::append(a, it.node1(), it.node2());
    netbridges::clear(b);
    ok &= netbridges::count(a) == 15 && netbridges::count(b) == 0 && netbridges::poolUsed() == 15;
    { int i = 1; for (auto it = netbridges::begin(a); it.valid(); it.next(), i++) { if (i <= 10) ok &= it.node1() == i; else ok &= it.node1() == 100 + (i - 10); } ok &= i == 16; }
    // shiftNets: a copy of the header must be detached, not cleared
    netStruct copy = a; netbridges::detach(copy);
    ok &= netbridges::count(a) == 15 && netbridges::poolUsed() == 15 && netbridges::count(copy) == 0;
    // exhaustion: capacity - 15 more appends succeed, the next fails, nothing else changes
    int room = netbridges::capacity() - netbridges::poolUsed(); netStruct c{};
    for (int i = 0; i < room; i++) ok &= netbridges::append(c, 7, 8);
    ok &= !netbridges::append(c, 7, 8) && netbridges::count(c) == room && netbridges::poolUsed() == netbridges::capacity();
    netbridges::clear(c); ok &= netbridges::poolUsed() == 15;
    ok &= netbridges::append(c, 7, 8) && netbridges::poolUsed() == 16;
    netbridges::clear(a); netbridges::clear(c); ok &= netbridges::poolUsed() == 0;
    // the routing state's own sizes, so the memory this bought stays bought
    printf("  sizeof(netStruct)=%zu sizeof(pathStruct)=%zu sizeof(NetBridgePool)=%zu MAX_NODES=%d capacity=%d\n", sizeof(netStruct), sizeof(pathStruct), sizeof(NetBridgePool), MAX_NODES, netbridges::capacity());
    // ARM: enums are 1 byte and pointers 4 -> pathStruct 40, netStruct 104.
    // The host build pays 4-byte enums and 8-byte pointers on top of that.
    ok &= sizeof(pathStruct) <= 40 + 4 * (sizeof(enum pathType) - 1)
       && sizeof(netStruct) <= 104 + 4 * (sizeof(void*) - 4)
       && netbridges::capacity() >= 2 * MAX_BRIDGES;
    printf("  %s\n", ok ? "PASS" : "FAIL"); fails += !ok;
    netbridges::resetAll();
  }
#endif
  // Corner reached only through lanes the same net already owns: two GND rows
  // on EVERY breadboard chip take both its I and J lanes (17 bridges, under
  // the cap), so 31 can only hop through a lane GND already holds. The L-hop
  // search used to demand a virgin lane and gave up.
  { NetDef g{1, {GND}, {}}; for (int r : {2, 3, 9, 10, 16, 17, 23, 24, 33, 34, 40, 41, 47, 48, 54, 55}) { g.nodes.push_back(r); g.bridges.push_back({GND, r}); }
    g.nodes.push_back(31); g.bridges.push_back({GND, 31});
    fails += !runCase("5: GND on 2 rows of every chip + 31 (corner via same-net lanes)", {g}, verbose); }
  // Batching (connect(refresh=False) ... leds_flush()): the hold only skips
  // core 1's LED render; every call still rebuilds and posts the same
  // crosspoint send. So the crossbar after N calls must equal ONE rebuild of
  // the same netlist - route each prefix of the list (a rebuild per call, the
  // way fast_connect does) and require the final digest to match the
  // all-at-once digest. A hold that changed routing would show up here.
  {
    std::vector<std::pair<int,int>> adds = {{GND, 4}, {GND, 5}, {SUPPLY_3V3, 20}, {20, 21}, {ADC0, 31}, {GND, 30}, {DAC0, 60}, {21, 22}};
    auto build = [&](size_t n) {
      std::vector<NetDef> defs; NetDef g{1, {GND}, {}}, v{6, {SUPPLY_3V3}, {}}, a{7, {ADC0}, {}}, d{8, {DAC0}, {}};
      for (size_t i = 0; i < n; i++) { auto b = adds[i];
        NetDef* t = (b.first == GND) ? &g : (b.first == SUPPLY_3V3 || b.first == 20 || b.first == 21) ? &v : (b.first == ADC0) ? &a : &d;
        t->bridges.push_back(b); t->nodes.push_back(b.second); }
      for (NetDef* t : {&g, &v, &a, &d}) if (!t->bridges.empty()) defs.push_back(*t);
      return defs;
    };
    fflush(stdout); int saved = dup(1); FILE* tmp = tmpfile(); dup2(fileno(tmp), 1);
    std::string stepwise; for (size_t n = 1; n <= adds.size(); n++) { runCase("batch step", build(n), false); stepwise = lastDigest; }
    runCase("batch all", build(adds.size()), false); std::string once = lastDigest;
    fflush(stdout); dup2(saved, 1); close(saved); fclose(tmp);
    bool ok = stepwise == once && !once.empty();
    printf("\n=== 8: one rebuild per call == one rebuild of the batch (crosspoints) ===\n  %zu calls -> %s\n  %s\n", adds.size(), once.c_str(), ok ? "PASS" : "FAIL");
    fails += !ok;
  }
  // connect_many(): k disconnects + k connects applied to the netlist, then ONE
  // rebuild, must close the same crosspoints as k rebuilds of the sequential
  // edits. The firmware's rebuild is a function of connections.bridges[] in
  // its stored order, so the model is the bridge list itself: sequential =
  // erase(d_i) then push_back(c_i) with a rebuild per pair; batch = all
  // erases, all push_backs, one rebuild. (3V3 on rows 1..k moved to 31..30+k,
  // the fixture's frame.)
  for (int k : {1, 4, 8, 24}) {
    std::vector<std::pair<int,int>> seq, batch;
    for (int r = 1; r <= k; r++) { seq.push_back({SUPPLY_3V3, r}); }
    batch = seq;
    auto toDefs = [&](const std::vector<std::pair<int,int>>& br) {
      NetDef v{6, {SUPPLY_3V3}, {}}; for (auto& b : br) { v.bridges.push_back(b); v.nodes.push_back(b.second); }
      return std::vector<NetDef>{v};
    };
    auto erasePair = [](std::vector<std::pair<int,int>>& br, std::pair<int,int> p) {
      for (size_t i = 0; i < br.size(); i++) if (br[i] == p) { br.erase(br.begin() + i); return; } };
    fflush(stdout); int saved = dup(1); FILE* tmp = tmpfile(); dup2(fileno(tmp), 1);
    std::string seqDigest;
    for (int r = 1; r <= k; r++) {
      erasePair(seq, {SUPPLY_3V3, r}); runCase("seq d", toDefs(seq), false);
      seq.push_back({SUPPLY_3V3, 30 + r}); runCase("seq c", toDefs(seq), false); seqDigest = lastDigest;
    }
    for (int r = 1; r <= k; r++) erasePair(batch, {SUPPLY_3V3, r});
    for (int r = 1; r <= k; r++) batch.push_back({SUPPLY_3V3, 30 + r});
    runCase("batch", toDefs(batch), false); std::string batchDigest = lastDigest;
    fflush(stdout); dup2(saved, 1); close(saved); fclose(tmp);
    bool ok = seqDigest == batchDigest && !batchDigest.empty() && seq == batch;
    printf("\n=== 9: connect_many k=%d (k disconnect + k connect, one rebuild) == sequential ===\n  %s\n", k, ok ? "PASS" : "FAIL");
    if (!ok) { printf("  seq:   %s\n  batch: %s\n", seqDigest.c_str(), batchDigest.c_str()); }
    fails += !ok;
  }
  // 30 two-row nets, top row r to bottom row 30+r: the densest plain netlist
  // (every chip lane in use); every bridge the rule calls clean is closed.
  // Like case 6 the crossbar may leave a link unrouted (it does: one of 30);
  // the assertions are no short and no clean-but-open verdict, and the rule
  // must flag exactly the links the model shows open.
  { std::vector<NetDef> links; for (int r = 1; r <= 30; r++) links.push_back({6 + r - 1, {r, 30 + r}, {{r, 30 + r}}});
    long u0 = healthUnrouted, j0 = healthUnroutedButJoined;
    fflush(stdout); int saved = dup(1); FILE* tmp = tmpfile(); dup2(fileno(tmp), 1);
    runCase("10: 30 top-bottom links (r <-> 30+r)", links, verbose);
    fflush(stdout); dup2(saved, 1); close(saved); rewind(tmp);
    int shorts = 0, unrouted = 0, healthFail = 0; char line[4096];
    while (fgets(line, sizeof line, tmp)) { if (strstr(line, "SHORTED") || strstr(line, "touches")) shorts++; if (strstr(line, "NOT connected")) unrouted++; if (strstr(line, "FAIL health")) healthFail++; }
    fclose(tmp);
    long ruleUnrouted = healthUnrouted - u0, ruleJoined = healthUnroutedButJoined - j0;
    // two-node nets: an unrouted bridge IS an open link, so the counts must agree
    bool ok = shorts == 0 && healthFail == 0 && ruleJoined == 0 && ruleUnrouted == unrouted;
    printf("\n=== 10: 30 top-bottom links (r <-> 30+r) ===\n  open links (model) %d, unrouted (rule) %ld, shorts %d, clean-but-open %d  %s\n", unrouted, ruleUnrouted, shorts, healthFail, ok ? "PASS" : "FAIL");
    fails += !ok; }
  // sanity: the thing that works on hardware
  // P: the peripheral nodes the 2026-09-11 bench used (the special-function
  // X pins of chips I/J/K/L per the rev 2 PCB netlist: 5V on J14/L14, I+/I- on
  // L1/L0, DAC0 on I12/L7, DAC1 on J12/L6, ADC0-3 on I13/J13/K15 + L2-L5).
  fails += !runCase("P1: 5V-8 (supply node on J/L)", {{6, {SUPPLY_5V, 8}, {{SUPPLY_5V, 8}}}}, verbose);
  fails += !runCase("P2: 3V3-I+ ; I- -12 ; GND-13 (INA loop)", {{6, {SUPPLY_3V3, ISENSE_PLUS}, {{SUPPLY_3V3, ISENSE_PLUS}}}, {7, {ISENSE_MINUS, 12}, {{ISENSE_MINUS, 12}}}, {1, {GND, 13}, {{GND, 13}}}}, verbose);
  fails += !runCase("P3: DAC0-20 + ADC0-20 (DAC read back)", {{6, {DAC0, 20, ADC0}, {{DAC0, 20}, {ADC0, 20}}}}, verbose);
  fails += !runCase("P4: DAC1-40 + ADC3-40", {{6, {DAC1, 40, ADC3}, {{DAC1, 40}, {ADC3, 40}}}}, verbose);
  fails += !runCase("P5: GND-ADC3 ; 3V3-9-ADC2 ; 5V-12-ADC1 (SF to ADC through a row)",
                    {{1, {GND, ADC3}, {{GND, ADC3}}}, {6, {SUPPLY_3V3, 9, ADC2}, {{SUPPLY_3V3, 9}, {9, ADC2}}}, {7, {SUPPLY_5V, 12, ADC1}, {{SUPPLY_5V, 12}, {12, ADC1}}}}, verbose);
  // KNOWN (reported 2026-09-11, router untouched): a supply DIRECTLY to ADC1
  // or ADC2 with no row in the net is left unrouted - the I->A->K three-chip
  // path keeps a -2 Y position (see the v trace). Through a row it routes
  // (P5). Counted separately so the harness stays green while it is open.
  int known = 0;
  known += !runCase("K1 (known): 3V3-ADC2 direct", {{6, {SUPPLY_3V3, ADC2}, {{SUPPLY_3V3, ADC2}}}}, verbose);
  known += !runCase("K2 (known): GND-ADC2 direct", {{1, {GND, ADC2}, {{GND, ADC2}}}}, verbose);
  known += !runCase("K3 (known): 3V3-ADC1 direct", {{6, {SUPPLY_3V3, ADC1}, {{SUPPLY_3V3, ADC1}}}}, verbose);
  printf("\n%d known-open direct SF->ADC1/ADC2 cases still unrouted (not counted)\n", known);
  fails += !runCase("S: 3V3-5 + 5-1 (works on hw)", {{6, {SUPPLY_3V3, 5, 1}, {{SUPPLY_3V3, 5}, {5, 1}}}}, verbose);
  printf("\nPathHealth rule over all cases: %ld bridges, %ld unrouted, %ld of those still joined via other bridges, 0 clean-but-open (asserted)\n", healthBridges, healthUnrouted, healthUnroutedButJoined);
  printf("\n%d failing cases\n", fails);
  return fails ? 1 : 0;
}
