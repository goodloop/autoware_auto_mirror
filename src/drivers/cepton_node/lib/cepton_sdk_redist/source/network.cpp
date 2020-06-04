#include "cepton_sdk/network.hpp"

#include "cepton_sdk/core.hpp"
#include "cepton_sdk/sensor.hpp"
#include "cepton_sdk_api.hpp"
#include "cepton_sdk_util.hpp"

using asio::ip::udp;

namespace cepton_sdk {

// -----------------------------------------------------------------------------
// SocketListener
// -----------------------------------------------------------------------------
SocketListener::SocketListener(uint16_t port)
    : m_socket(m_io_service, udp::v4()) {
  m_socket.set_option(asio::socket_base::reuse_address(true));
  m_socket.bind(udp::endpoint(udp::v4(), port));
}

SocketListener::~SocketListener() { stop(); }

void SocketListener::run() {
  listen();
  m_io_service.run();
}

void SocketListener::stop() {
  std::lock_guard<std::mutex> lock(m_mutex);
  try {
    // Cancel asio calls (This will throw system error in windows sometimes)
    m_socket.cancel();

    // Shutdown is required by windows OS, may throw system error in OSX
    m_socket.shutdown(udp::socket::shutdown_both);
    m_socket.close();
  } catch (const std::system_error &) {
    // Tolerate system error of these types:
    // OSX: "shutdown: Socket is not connected"
    // Windows: "cancel: The file handle supplied is not valid."
  }
  m_io_service.stop();
  m_io_service.reset();
}

void SocketListener::listen() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_socket.async_receive_from(
      asio::buffer(m_buffer), m_end_point,
      [this](const asio::error_code &error, std::size_t buffer_size) {
        if (buffer_size == 0) return;
        if (error == asio::error::operation_aborted) return;
        const CeptonSensorHandle handle =
            m_end_point.address().to_v4().to_ulong();
        callback(error, handle, buffer_size, m_buffer.data());
        listen();
      });
}

// -----------------------------------------------------------------------------
// NetworkManager
// -----------------------------------------------------------------------------
NetworkManager network_manager;

void NetworkManager::initialize() {
  deinitialize();
  if (sdk_manager.has_control_flag(CEPTON_SDK_CONTROL_DISABLE_NETWORK)) return;

  m_is_running = true;

  m_listener.reset(new SocketListener(m_port));
  m_listener->callback.listen([&](const asio::error_code &error,
                                  CeptonSensorHandle handle, int buffer_size,
                                  const uint8_t *const buffer) {
    if (error.value()) {
      callback_manager.emit_error(CEPTON_NULL_HANDLE,
                                  CEPTON_ERROR_COMMUNICATION,
                                  error.message().c_str(), &error);
      return;
    }

    thread_local std::shared_ptr<util::LargeObjectPool<Packet>> packet_pool;
    if (!packet_pool) packet_pool.reset(new util::LargeObjectPool<Packet>());
    auto packet = packet_pool->get();
    packet->handle = handle;
    packet->timestamp = util::get_timestamp_usec();
    packet->buffer.clear();
    packet->buffer.reserve(buffer_size);
    packet->buffer.insert(packet->buffer.end(), buffer, buffer + buffer_size);
    if (m_packets.size() > 1000) {
#ifdef CEPTON_INTERNAL
    // api::log_error(
    //    SensorError(CEPTON_ERROR_COMMUNICATION, "Packet queue full!"),
    //    "Network failed!");
#endif
      m_packets.clear();
    }
    m_packets.push(packet);
  });
  m_listener_thread.reset(new std::thread([&]() { m_listener->run(); }));

  m_worker_thread.reset(new std::thread([&]() {
    while (true) {
      const auto packet = m_packets.pop(0.01f);
      if (!m_is_running) break;
      if (!packet) continue;
      callback_manager.network_cb.emit(packet->handle, packet->timestamp,
                                       packet->buffer.data(),
                                       packet->buffer.size());
      sensor_manager.handle_network_receive(packet->handle, packet->timestamp,
                                            packet->buffer.size(),
                                            packet->buffer.data());
    }
  }));

  m_is_initialized = true;
}

void NetworkManager::deinitialize() {
  if (!m_is_initialized) return;

  m_is_running = false;
  m_listener->stop();
  if (m_listener_thread) {
    m_listener_thread->join();
    m_listener_thread.reset();
  }
  if (m_listener) {
    m_listener.reset();
  }
  if (m_worker_thread) {
    m_worker_thread->join();
    m_worker_thread.reset();
  }

  m_is_initialized = false;
}

uint16_t NetworkManager::get_port() const { return m_port; }

SensorError NetworkManager::set_port(uint16_t port) {
  const bool is_initialized = m_is_initialized;
  deinitialize();
  m_port = port;
  if (is_initialized) initialize();
  return CEPTON_SUCCESS;
}

}  // namespace cepton_sdk
