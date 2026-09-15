// Test-only interposer: allow the unmodified SDK and fake robot to share one host.
// The SDK otherwise binds all local addresses at the robot's source port.
#include <arpa/inet.h>
#include <dlfcn.h>
#include <sys/socket.h>

extern "C" int bind(int fd, const sockaddr * address, socklen_t length)
{
  using Bind = int (*)(int, const sockaddr *, socklen_t);
  static auto real_bind = reinterpret_cast<Bind>(dlsym(RTLD_NEXT, "bind"));
  if (address->sa_family == AF_INET && length == sizeof(sockaddr_in)) {
    auto local = *reinterpret_cast<const sockaddr_in *>(address);
    // Dedicated smoke-test port only; never alter DDS or other application binds.
    if (local.sin_port == htons(30283) && local.sin_addr.s_addr == htonl(INADDR_ANY)) {
      inet_pton(AF_INET, "127.0.0.1", &local.sin_addr);
      return real_bind(fd, reinterpret_cast<const sockaddr *>(&local), sizeof(local));
    }
  }
  return real_bind(fd, address, length);
}
