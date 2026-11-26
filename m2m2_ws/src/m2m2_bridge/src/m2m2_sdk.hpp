// Interface simples para o Slamware SDK: compilado a parte em C++14
// (o SDK usa especificacao de excecao dinamica, proibida em C++17).
#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace m2m2 {

struct Pose { double x, y, yaw; };
struct Ponto { float dist, ang; bool valido; };
struct Mapa {
  double origem_x, origem_y, resolucao;
  int largura, altura;
  std::vector<uint8_t> dados;  // 0 desconhecido, 1..127 prob. de ocupacao (~127 = parede)
};

class Dispositivo {
public:
  Dispositivo();
  ~Dispositivo();
  // todas lancam std::runtime_error em falha de comunicacao
  std::string conectar(const std::string& host, int porta);  // retorna versao do firmware
  bool conectado() const;
  void desconectar();
  Pose pose();
  std::vector<Ponto> scan();
  bool mapa(Mapa& out);  // false se area vazia
  bool limpar_mapa();
  bool mapeando(bool ligar);
  bool salvar(const std::string& arquivo, std::string& erro);
  bool carregar(const std::string& arquivo, std::string& erro);
  int qualidade_localizacao();
  bool definir_pose(const Pose& p);  // pose do SENSOR no mapa

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace m2m2
