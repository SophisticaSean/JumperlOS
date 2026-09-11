#pragma once
struct RoutingConfig { int stack_paths=1; int stack_rails=2; int stack_dacs=1; int stack_adcs=0; int stack_gpio=0; };
struct JumperlessConfig { RoutingConfig routing; };
extern JumperlessConfig jumperlessConfig;
