#include <iostream>
#include <fstream>
#include <map>
#include <regex>
#include <string>
#include <string_view>
#include <cinttypes>

using std::map;
using std::string;
using std::regex;
using std::regex_match;
using std::regex_search;
using std::smatch;
using std::getline;
using std::cin;
using std::string_view;
using std::optional;

template<typename T>
string_view make_sv(const T &v)
{
  return string_view(static_cast<const char *>(v.first.base()),  //TODO: UD?
                          v.second - v.first);
}

struct rftemp1_msg
{
  uint64_t id;
  uint32_t sample;
  double temperature;
  double voltage;

  //bool operator ==(const rftemp1_msg &) const & = default; :(
  bool operator ==(const rftemp1_msg &b) const &
  {
    return std::tie(id, temperature, voltage, sample) == std::tie(b.id, b.temperature, b.voltage, b.sample);
  }
};


string make_insert(const rftemp1_msg &msg)
{
  char buffer[1024];
  sprintf(buffer, "INSERT INTO temp_sensor_value (id, temperature) values (%" PRId64 ", %lf);",
         msg.id, msg.temperature);
  return buffer;
}

optional<rftemp1_msg> parse_rftemp1(const map<string, string> values)
{
  if(!values.count("DALLASID") || !values.count("TEMP") || !values.count("VOLT") || !values.count("SAMPLE"))
    return {};

  rftemp1_msg msg;

  if(!sscanf(values.at("DALLASID").c_str(), "%" SCNx64, &msg.id))
    return {};
  if(!sscanf(values.at("TEMP").c_str(), "%lf", &msg.temperature))
    return {};
  if(!sscanf(values.at("VOLT").c_str(), "%lf", &msg.voltage))
    return {};
  if(!sscanf(values.at("SAMPLE").c_str(), "%" SCNu32, &msg.sample))
    return {};
  
  return msg;
}

int main()
{
  const regex rx_msg("^MSG;([A-Z]+:-?[0-9A-Z.]+;)+$");
  const regex rx_part("([A-Z]+):(-?[0-9A-Z.]+);");

  map<uint64_t, rftemp1_msg> last_msg;
  for(string line; getline(cin, line); )
  {
    smatch match;
    map<string, string> values;
  
    for(auto s = line.cbegin(); regex_search(s, line.cend(), match, rx_part); s = match.suffix().first)
      values[match[1].str()] = match[2].str();

    auto msg = parse_rftemp1(values);

    if(msg)
    {
      if(last_msg[msg->id] == *msg)
        continue;
      last_msg[msg->id] = *msg;
      std::cout << make_insert(*msg) << "\n";
    }
  }
  return 0;
}
