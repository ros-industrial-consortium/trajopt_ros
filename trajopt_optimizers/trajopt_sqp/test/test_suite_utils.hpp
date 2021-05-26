#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

inline std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://trajopt") == 0)
  {
    mod_url.erase(0, std::string("package://trajopt").length());
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TRAJOPT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}
