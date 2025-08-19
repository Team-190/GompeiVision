#include "util/LogRedirect.h"

LogRedirect::LogRedirect(const std::string& filename) {
  m_logFile.open(filename);
  m_pOldCoutBuf = std::cout.rdbuf();
  m_pOldCerrBuf = std::cerr.rdbuf();
  std::cout.rdbuf(m_logFile.rdbuf());
  std::cerr.rdbuf(m_logFile.rdbuf());
}

LogRedirect::~LogRedirect() {
  std::cout.rdbuf(m_pOldCoutBuf);
  std::cerr.rdbuf(m_pOldCerrBuf);
}
